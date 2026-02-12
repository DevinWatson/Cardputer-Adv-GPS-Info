"""
Serial Screenshot Capture for M5Cardputer GPS Info
Listens on COM port for screenshot data sent when Enter is pressed.
Saves as BMP files in the current directory.

Usage: python utils/serial_screenshot.py [COM_PORT]
Default port: COM5
"""

import serial
import sys
import os
import struct
import time

def rgb565_to_rgb888(c):
    r = ((c >> 11) & 0x1F) * 255 // 31
    g = ((c >> 5) & 0x3F) * 255 // 63
    b = (c & 0x1F) * 255 // 31
    return (r, g, b)

def save_bmp(filename, width, height, pixels):
    row_bytes = width * 3
    pad = (4 - (row_bytes % 4)) % 4
    stride = row_bytes + pad
    img_size = stride * height
    file_size = 54 + img_size

    with open(filename, 'wb') as f:
        # BMP header
        f.write(b'BM')
        f.write(struct.pack('<I', file_size))
        f.write(struct.pack('<HH', 0, 0))
        f.write(struct.pack('<I', 54))
        # DIB header
        f.write(struct.pack('<I', 40))
        f.write(struct.pack('<i', width))
        f.write(struct.pack('<i', height))
        f.write(struct.pack('<HH', 1, 24))
        f.write(struct.pack('<I', 0))
        f.write(struct.pack('<I', img_size))
        f.write(struct.pack('<i', 2835))
        f.write(struct.pack('<i', 2835))
        f.write(struct.pack('<I', 0))
        f.write(struct.pack('<I', 0))
        # Pixel data (bottom-up)
        for y in range(height - 1, -1, -1):
            row = b''
            for x in range(width):
                r, g, b = pixels[y * width + x]
                row += struct.pack('BBB', b, g, r)
            row += b'\x00' * pad
            f.write(row)

def main():
    port = sys.argv[1] if len(sys.argv) > 1 else 'COM5'
    baud = 115200

    print(f"Opening {port} at {baud} baud...")
    try:
        ser = serial.Serial()
        ser.port = port
        ser.baudrate = baud
        ser.timeout = 60
        ser.dtr = False
        ser.rts = False
        ser.open()
        time.sleep(0.1)
        ser.dtr = True
    except serial.SerialException as e:
        print(f"Error: {e}")
        print("Make sure no other program (like pio monitor) is using the port.")
        sys.exit(1)

    print(f"Listening for screenshots on {port}...")
    print("Press Enter on the Cardputer to capture a screenshot.")
    print("Press Ctrl+C to quit.\n")

    screenshot_num = 1
    save_dir = os.getcwd()

    try:
        while True:
            try:
                raw = ser.readline()
            except serial.SerialException:
                time.sleep(0.1)
                continue
            if not raw:
                continue
            line = raw.decode('ascii', errors='ignore').strip()
            if not line:
                continue

            # Pass through normal serial output
            if "SCREENSHOT_START" not in line:
                print(f"[serial] {line}")
                continue

            print("\n>> SCREENSHOT_START detected!")
            raw_dims = ser.readline()
            dims_line = raw_dims.decode('ascii', errors='ignore').strip()
            print(f"   Dimensions line: '{dims_line}'")
            dims = dims_line.split()
            if len(dims) != 2:
                print(f"   ERROR: Bad dimensions, skipping")
                continue

            w, h = int(dims[0]), int(dims[1])
            print(f"   Size: {w}x{h}, reading {h} rows...")
            pixels = []
            bad_rows = 0

            for row_idx in range(h):
                hex_line = ser.readline().decode('ascii', errors='ignore').strip()
                expected = w * 4
                if len(hex_line) != expected:
                    if row_idx < 3 or row_idx == h - 1:
                        print(f"   Row {row_idx}: got {len(hex_line)} chars (expected {expected})")
                    bad_rows += 1
                    hex_line = hex_line.ljust(expected, '0')[:expected]
                for i in range(0, w * 4, 4):
                    try:
                        c = int(hex_line[i:i+4], 16)
                    except ValueError:
                        c = 0
                    pixels.append(rgb565_to_rgb888(c))
                if row_idx % 30 == 0:
                    print(f"   Row {row_idx}/{h}...")

            end = ser.readline().decode('ascii', errors='ignore').strip()
            print(f"   End marker: '{end}'")

            if bad_rows:
                print(f"   Warning: {bad_rows}/{h} rows had unexpected length")

            total_pixels = w * h
            print(f"   Got {len(pixels)} pixels (expected {total_pixels})")

            if len(pixels) == total_pixels:
                while os.path.exists(os.path.join(save_dir, f"scr_{screenshot_num:04d}.bmp")):
                    screenshot_num += 1
                filename = os.path.join(save_dir, f"scr_{screenshot_num:04d}.bmp")
                save_bmp(filename, w, h, pixels)
                print(f"   SAVED: {filename}")
                screenshot_num += 1
            else:
                print(f"   ERROR: Pixel count mismatch, not saving")

    except KeyboardInterrupt:
        print("\nDone.")
    finally:
        ser.close()

if __name__ == '__main__':
    main()
