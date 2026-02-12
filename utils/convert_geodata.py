#!/usr/bin/env python3
"""
Convert Natural Earth GeoJSON data to PROGMEM C arrays for ESP32 GPS map.

Uses 10m (high-res) data where available, falls back to 50m.
Adds bounding-box headers (0x7FFE marker) per segment for fast skip
when zoomed in.

Output arrays:
  - coastData[]   (coastlines, green)
  - borderData[]  (country borders, dim yellow)
  - stateData[]   (state/province outlines, dim cyan)
  - riverData[]   (rivers, blue)
  - lakeData[]    (lakes, cyan)
  - cityList[]    (cities, yellow dots)

Data format (coordinates are degrees * COORD_SCALE):
  0x7FFE, minLat, maxLat, minLon, maxLon,  // bbox header
  lat, lon, lat, lon, ...,                  // coordinate pairs
  0x7FFF,                                   // segment break
  ...
  0x7FFF                                    // end marker
"""

import json
import math
import os
import sys

sys.setrecursionlimit(50000)

PROJECT_ROOT = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))
GEODATA_DIR = os.path.join(PROJECT_ROOT, "geodata")
OUTPUT_FILE = os.path.join(PROJECT_ROOT, "src", "map_data.h")

# Coordinate scale: degrees * COORD_SCALE stored as int16_t.
# *10 = 0.1° resolution (~11km), *100 = 0.01° (~1.1km).
# int16_t range: -32766..32766. At *100: lat max 90°=9000, lon max 180°=18000. Fits.
COORD_SCALE = 100

# Flash budget: 3,342,336 bytes total. Current firmware ~567KB.
# Target: ~80% flash = 2,674KB. Available for data: ~2,100KB.
# But we need rendering to stay under ~200ms at world zoom.
# With bbox optimization, zoomed rendering is fast regardless.
# At world zoom: all segments visible, ~150 cycles/point.
# Budget per array (int16_t values, each = 2 bytes):
MAX_COAST_VALUES  = 500000   # ~1.0MB - coastlines are the star
MAX_BORDER_VALUES = 200000   # ~400KB
MAX_STATE_VALUES  = 300000   # ~600KB - full polygon outlines
MAX_RIVER_VALUES  = 100000   # ~200KB
MAX_LAKE_VALUES   = 100000   # ~200KB

# Low-res arrays for 3D globe (32px radius, 1px ≈ 5.6° arc)
MAX_COAST_LOW_VALUES = 8000   # ~16KB - simplified coastlines for globe
MAX_BORDER_LOW_VALUES = 5000  # ~10KB - simplified borders for globe
GLOBE_TOLERANCE = 500  # 5° in COORD_SCALE units (~556km)
MIN_GLOBE_BBOX_SPAN = 500  # 5° in centidegrees — drop segments smaller than this

# City config
MAX_CITIES = 500
MIN_CITY_POP = 200000  # Include cities with 200K+ population


def douglas_peucker_iterative(points, tolerance):
    """Iterative Douglas-Peucker to avoid recursion limit."""
    if len(points) <= 2:
        return points

    # Use a stack-based approach
    keep = [False] * len(points)
    keep[0] = True
    keep[-1] = True

    stack = [(0, len(points) - 1)]
    while stack:
        start, end = stack.pop()
        if end - start <= 1:
            continue

        dmax = 0
        idx = start
        p1 = points[start]
        p2 = points[end]

        for i in range(start + 1, end):
            d = point_line_distance(points[i], p1, p2)
            if d > dmax:
                dmax = d
                idx = i

        if dmax > tolerance:
            keep[idx] = True
            stack.append((start, idx))
            stack.append((idx, end))

    return [p for i, p in enumerate(points) if keep[i]]


def point_line_distance(p, p1, p2):
    """Perpendicular distance from point p to line p1-p2."""
    dx = p2[0] - p1[0]
    dy = p2[1] - p1[1]
    denom = dx * dx + dy * dy
    if denom == 0:
        return math.sqrt((p[0] - p1[0])**2 + (p[1] - p1[1])**2)
    t = max(0, min(1, ((p[0] - p1[0]) * dx + (p[1] - p1[1]) * dy) / denom))
    proj_x = p1[0] + t * dx
    proj_y = p1[1] + t * dy
    return math.sqrt((p[0] - proj_x)**2 + (p[1] - proj_y)**2)


def load_geojson(filename):
    """Load a GeoJSON file."""
    filepath = os.path.join(GEODATA_DIR, filename)
    if not os.path.exists(filepath):
        print(f"  WARNING: {filename} not found, skipping")
        return None
    size_mb = os.path.getsize(filepath) / 1024 / 1024
    print(f"  Loading {filename} ({size_mb:.1f} MB)...")
    with open(filepath, 'r', encoding='utf-8') as f:
        return json.load(f)


def load_best(name_10m, name_50m):
    """Load 10m data if available, else 50m."""
    gj = load_geojson(name_10m)
    if gj is None:
        gj = load_geojson(name_50m)
    return gj


def extract_linestrings(geojson):
    """Extract all linestring coordinates from a GeoJSON feature collection."""
    segments = []
    if geojson is None:
        return segments
    for feature in geojson.get('features', []):
        geom = feature.get('geometry')
        if geom is None:
            continue
        gtype = geom.get('type', '')
        coords = geom.get('coordinates', [])
        if gtype == 'LineString':
            segments.append(coords)
        elif gtype == 'MultiLineString':
            for line in coords:
                segments.append(line)
        elif gtype == 'Polygon':
            for ring in coords:
                segments.append(ring)
        elif gtype == 'MultiPolygon':
            for polygon in coords:
                for ring in polygon:
                    segments.append(ring)
    return segments


def extract_cities(geojson, max_cities=MAX_CITIES, min_pop=MIN_CITY_POP):
    """Extract city points from populated places GeoJSON."""
    cities = []
    if geojson is None:
        return cities
    for feature in geojson.get('features', []):
        props = feature.get('properties', {})
        geom = feature.get('geometry', {})
        if geom.get('type') != 'Point':
            continue
        coords = geom.get('coordinates', [])
        if len(coords) < 2:
            continue

        lon, lat = coords[0], coords[1]
        pop = props.get('pop_max', 0) or props.get('pop_min', 0) or 0
        # Use ASCII name to avoid encoding issues
        name = props.get('nameascii', '') or props.get('name', '') or ''

        if not name or pop < min_pop:
            continue

        # Truncate to 11 chars, sanitize
        name = ''.join(c for c in name if 32 <= ord(c) < 127)
        name = name[:11]
        name = name.replace('"', "'").replace('\\', '')

        if not name:
            continue

        cities.append((lat, lon, name, pop))

    cities.sort(key=lambda c: -c[3])
    return cities[:max_cities]


def filter_small_segments(segments, min_span):
    """Drop segments whose lat AND lon extent are both below min_span (centidegrees).

    These tiny features (small islands, short border stubs) are invisible on
    the 32px-radius globe where 1 pixel ≈ 5.6° of arc.
    """
    kept = []
    for seg in segments:
        if len(seg) < 2:
            continue
        # Convert to centidegrees and measure bbox span
        lats = [coord[1] * COORD_SCALE for coord in seg]
        lons = [coord[0] * COORD_SCALE for coord in seg]
        lat_span = max(lats) - min(lats)
        lon_span = max(lons) - min(lons)
        if lat_span >= min_span or lon_span >= min_span:
            kept.append(seg)
    return kept


def segments_to_progmem(segments, tolerance, max_values, name):
    """Convert line segments to PROGMEM int16_t array with bbox headers."""
    values = []
    total_input_points = sum(len(s) for s in segments)
    total_output_points = 0
    segments_added = 0

    for seg in segments:
        if len(seg) < 2:
            continue

        # Convert to lat*COORD_SCALE, lon*COORD_SCALE
        points_10 = []
        for coord in seg:
            lon, lat = coord[0], coord[1]
            lat10 = int(round(lat * COORD_SCALE))
            lon10 = int(round(lon * COORD_SCALE))
            lat10 = max(-32766, min(32766, lat10))
            lon10 = max(-32766, min(32766, lon10))
            points_10.append((lat10, lon10))

        # Remove consecutive duplicates
        deduped = [points_10[0]]
        for i in range(1, len(points_10)):
            if points_10[i] != points_10[i-1]:
                deduped.append(points_10[i])
        points_10 = deduped

        if len(points_10) < 2:
            continue

        # Simplify
        if tolerance > 0 and len(points_10) > 2:
            simplified = douglas_peucker_iterative(points_10, tolerance)
        else:
            simplified = points_10

        if len(simplified) < 2:
            continue

        # Calculate bounding box
        lats = [p[0] for p in simplified]
        lons = [p[1] for p in simplified]
        bbox = [min(lats), max(lats), min(lons), max(lons)]

        # Check if adding this segment fits
        # bbox header: 0x7FFE + 4 values = 5 values
        # points: len(simplified) * 2 values
        # break: 1 value
        needed = 5 + len(simplified) * 2 + 1
        if len(values) + needed + 1 > max_values:
            break

        # Add bbox header
        values.append(0x7FFE)  # bbox marker
        values.extend(bbox)    # minLat, maxLat, minLon, maxLon

        # Add points
        for lat10, lon10 in simplified:
            values.append(lat10)
            values.append(lon10)
        values.append(0x7FFF)  # segment break

        total_output_points += len(simplified)
        segments_added += 1

    values.append(0x7FFF)  # end marker

    print(f"  {name}: {total_input_points} input -> {total_output_points} output pts "
          f"({segments_added} segs), {len(values)} vals, {len(values)*2:,} bytes")
    return values


def format_array(values, array_name, comment):
    """Format int16_t values as a C PROGMEM array."""
    lines = []
    lines.append(f"// {comment}")
    lines.append(f"// {len(values)} values, {len(values)*2:,} bytes")
    lines.append(f"static const int16_t PROGMEM {array_name}[] = {{")

    per_line = 14
    for i in range(0, len(values), per_line):
        chunk = values[i:i+per_line]
        formatted = []
        for v in chunk:
            if v == 0x7FFF:
                formatted.append("0x7FFF")
            elif v == 0x7FFE:
                formatted.append("0x7FFE")
            else:
                formatted.append(str(v))
        lines.append("  " + ",".join(formatted) + ",")

    lines.append("};")
    return "\n".join(lines)


def format_cities(cities):
    """Format city data as a C PROGMEM array."""
    lines = []
    lines.append(f"// World cities ({len(cities)} entries)")
    lines.append("struct MapCity {")
    lines.append("  int16_t lat10;")
    lines.append("  int16_t lon10;")
    lines.append("  char name[12];")
    lines.append("};")
    lines.append("static const MapCity PROGMEM cityList[] = {")

    for lat, lon, name, pop in cities:
        latS = int(round(lat * COORD_SCALE))
        lonS = int(round(lon * COORD_SCALE))
        lines.append(f'  {{{latS},{lonS},"{name}"}},')

    lines.append("};")
    lines.append("#define NUM_CITIES (sizeof(cityList)/sizeof(cityList[0]))")
    return "\n".join(lines)


def auto_tune_tolerance(segments_dict, targets, max_values_dict):
    """Binary search for tolerance that fills flash budget."""
    results = {}

    for name, (segments, target_bytes, max_vals) in zip(
        segments_dict.keys(),
        zip(segments_dict.values(),
            [targets[k] for k in segments_dict.keys()],
            [max_values_dict[k] for k in segments_dict.keys()])
    ):
        if not segments:
            results[name] = ([], 0.0)
            continue

        # Binary search for tolerance
        lo, hi = 0.0, 20.0
        best_values = None
        best_tol = hi

        # First try with 0 tolerance (all points)
        test = segments_to_progmem(segments, 0.0, max_vals, f"{name}(tol=0)")
        test_bytes = len(test) * 2

        if test_bytes <= target_bytes:
            # All points fit! Use tolerance 0
            results[name] = (test, 0.0)
            print(f"  -> {name}: ALL points fit ({test_bytes:,} <= {target_bytes:,})")
            continue

        # Need simplification. Binary search.
        for _ in range(15):
            mid = (lo + hi) / 2
            test = segments_to_progmem(segments, mid, max_vals, f"{name}(tol={mid:.3f})")
            test_bytes = len(test) * 2

            if test_bytes <= target_bytes:
                best_values = test
                best_tol = mid
                hi = mid  # Try lower tolerance for more detail
            else:
                lo = mid  # Need more simplification

        if best_values is None:
            # Even max tolerance exceeds budget, just use max
            best_values = segments_to_progmem(segments, hi, max_vals, f"{name}(max)")
            best_tol = hi

        results[name] = (best_values, best_tol)
        print(f"  -> {name}: tol={best_tol:.3f}, {len(best_values)*2:,} bytes")

    return results


def main():
    print("=== Natural Earth to PROGMEM Converter ===")
    print(f"Target: fill ESP32-S3 flash to ~80%\n")

    # Load datasets - prefer 10m, fallback to 50m
    print("Loading GeoJSON files...")
    coast_gj = load_best("ne_10m_coastline.geojson", "ne_50m_coastline.geojson")
    border_gj = load_best("ne_10m_admin_0_boundary_lines_land.geojson",
                          "ne_50m_admin_0_boundary_lines_land.geojson")
    state_gj = load_best("ne_10m_admin_1_states_provinces.geojson",
                         "ne_50m_admin_1_states_provinces.geojson")
    river_gj = load_best("ne_10m_rivers_lake_centerlines.geojson",
                         "ne_50m_rivers_lake_centerlines.geojson")
    lake_gj = load_best("ne_10m_lakes.geojson", "ne_50m_lakes.geojson")
    cities_gj = load_geojson("ne_10m_populated_places_simple.geojson")

    # Extract segments
    print("\nExtracting geometry...")
    coast_segs = extract_linestrings(coast_gj)
    border_segs = extract_linestrings(border_gj)
    state_segs = extract_linestrings(state_gj)
    river_segs = extract_linestrings(river_gj)
    lake_segs = extract_linestrings(lake_gj)

    total_input = (sum(len(s) for s in coast_segs) +
                   sum(len(s) for s in border_segs) +
                   sum(len(s) for s in state_segs) +
                   sum(len(s) for s in river_segs) +
                   sum(len(s) for s in lake_segs))
    print(f"Total input points: {total_input:,}")

    # Flash budget allocation (bytes of PROGMEM data)
    # Total available: ~2,500KB (to reach ~80% flash)
    # Allocate proportionally, favoring coastlines
    TOTAL_BUDGET = 2400 * 1024  # 2.4MB
    budget = {
        'coast':  int(TOTAL_BUDGET * 0.38),  # ~912KB
        'border': int(TOTAL_BUDGET * 0.18),  # ~432KB
        'state':  int(TOTAL_BUDGET * 0.22),  # ~528KB - full polygon outlines
        'river':  int(TOTAL_BUDGET * 0.11),  # ~264KB
        'lake':   int(TOTAL_BUDGET * 0.11),  # ~264KB
    }

    max_vals = {
        'coast':  MAX_COAST_VALUES,
        'border': MAX_BORDER_VALUES,
        'state':  MAX_STATE_VALUES,
        'river':  MAX_RIVER_VALUES,
        'lake':   MAX_LAKE_VALUES,
    }

    segments_dict = {
        'coast': coast_segs,
        'border': border_segs,
        'state': state_segs,
        'river': river_segs,
        'lake': lake_segs,
    }

    print(f"\nAuto-tuning simplification to fill {TOTAL_BUDGET/1024:.0f}KB budget...")
    print(f"  Budget: coast={budget['coast']//1024}KB, border={budget['border']//1024}KB, "
          f"state={budget['state']//1024}KB, river={budget['river']//1024}KB, "
          f"lake={budget['lake']//1024}KB")

    results = auto_tune_tolerance(segments_dict, budget, max_vals)

    coast_values = results['coast'][0]
    border_values = results['border'][0]
    state_values = results['state'][0]
    river_values = results['river'][0]
    lake_values = results['lake'][0]

    # Generate low-res arrays for 3D globe rendering
    # Pre-filter tiny segments invisible at 32px radius
    coast_globe = filter_small_segments(coast_segs, MIN_GLOBE_BBOX_SPAN)
    border_globe = filter_small_segments(border_segs, MIN_GLOBE_BBOX_SPAN)
    print(f"\nGlobe pre-filter: coasts {len(coast_segs)}->{len(coast_globe)} segs, "
          f"borders {len(border_segs)}->{len(border_globe)} segs")
    print(f"Generating low-res globe arrays (tolerance={GLOBE_TOLERANCE})...")
    coast_low_values = segments_to_progmem(
        coast_globe, GLOBE_TOLERANCE, MAX_COAST_LOW_VALUES, "coastDataLow")
    border_low_values = segments_to_progmem(
        border_globe, GLOBE_TOLERANCE, MAX_BORDER_LOW_VALUES, "borderDataLow")

    # Cities
    print("\nProcessing cities...")
    cities = extract_cities(cities_gj)
    print(f"  {len(cities)} cities included")

    # Final stats
    array_bytes = (len(coast_values) + len(border_values) + len(state_values) +
                   len(river_values) + len(lake_values)) * 2
    globe_bytes = (len(coast_low_values) + len(border_low_values)) * 2
    city_bytes = len(cities) * 16
    total_bytes = array_bytes + globe_bytes + city_bytes
    firmware_base = 567000
    total_flash = firmware_base + total_bytes
    flash_pct = total_flash / 3342336 * 100

    print(f"\n{'='*60}")
    print(f"FINAL SUMMARY")
    print(f"{'='*60}")
    print(f"  coastData:    {len(coast_values)*2:>10,} bytes  (tol={results['coast'][1]:.3f})")
    print(f"  borderData:   {len(border_values)*2:>10,} bytes  (tol={results['border'][1]:.3f})")
    print(f"  stateData:    {len(state_values)*2:>10,} bytes  (tol={results['state'][1]:.3f})")
    print(f"  riverData:    {len(river_values)*2:>10,} bytes  (tol={results['river'][1]:.3f})")
    print(f"  lakeData:     {len(lake_values)*2:>10,} bytes  (tol={results['lake'][1]:.3f})")
    print(f"  coastDataLow: {len(coast_low_values)*2:>10,} bytes  (tol={GLOBE_TOLERANCE}, globe)")
    print(f"  borderDataLow:{len(border_low_values)*2:>10,} bytes  (tol={GLOBE_TOLERANCE}, globe)")
    print(f"  cityList:     {city_bytes:>10,} bytes  ({len(cities)} cities)")
    print(f"  {'-'*40}")
    print(f"  TOTAL DATA: {total_bytes:>10,} bytes  ({total_bytes/1024:.1f} KB)")
    print(f"  + firmware: {firmware_base:>10,} bytes")
    print(f"  = FLASH:    {total_flash:>10,} bytes  ({flash_pct:.1f}%)")

    # Rendering performance
    total_points = 0
    for vals in [coast_values, border_values, state_values, river_values, lake_values]:
        pts = sum(1 for i in range(0, len(vals)-1, 2)
                  if vals[i] not in (0x7FFF, 0x7FFE)) // 1  # rough estimate
        total_points += len(vals) // 3  # approx
    est_ms = total_points * 150 / 240000
    print(f"\n  Rendering: ~{total_points:,} points, ~{est_ms:.0f}ms worst case (world zoom)")
    print(f"  With bbox skip: <50ms when zoomed in")

    # Write output
    print(f"\nWriting {OUTPUT_FILE}...")
    with open(OUTPUT_FILE, 'w', encoding='utf-8') as f:
        f.write("// AUTO-GENERATED from Natural Earth data. Do not edit manually.\n")
        f.write(f"// Total map data: {total_bytes:,} bytes ({total_bytes/1024:.1f} KB)\n")
        f.write("// Generated by utils/convert_geodata.py\n")
        f.write(f"// Coordinate scale: degrees * {COORD_SCALE} stored as int16_t\n")
        f.write("// Data format: 0x7FFE=bbox header, 0x7FFF=segment break/end\n\n")
        f.write("#pragma once\n\n")
        f.write(f"#define COORD_SCALE {COORD_SCALE}\n\n")

        f.write(format_array(coast_values, "coastData",
                             "World coastlines") + "\n\n")
        f.write(format_array(border_values, "borderData",
                             "Country borders") + "\n\n")
        f.write(format_array(state_values, "stateData",
                             "State/province borders") + "\n\n")
        f.write(format_array(river_values, "riverData",
                             "Major rivers") + "\n\n")
        f.write(format_array(lake_values, "lakeData",
                             "Major lakes") + "\n\n")
        f.write(format_array(coast_low_values, "coastDataLow",
                             "Simplified coastlines for 3D globe") + "\n\n")
        f.write(format_array(border_low_values, "borderDataLow",
                             "Simplified country borders for 3D globe") + "\n\n")
        f.write(format_cities(cities) + "\n")

    file_size = os.path.getsize(OUTPUT_FILE)
    print(f"Output file: {file_size:,} bytes ({file_size/1024/1024:.1f} MB)")
    print("Done!")


if __name__ == "__main__":
    main()
