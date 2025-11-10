# Cardputer GPS Info

Cardputer GPS Info is a lightweight GPS information tool for the M5Stack Cardputer.  
It reads GPS data via UART and displays location, speed, course, satellite visibility, and sky plots directly on the Cardputer screen.

<img src="https://raw.githubusercontent.com/alcor55/Cardputer-GPS-Info/main/1.jpeg" alt="Screenshot 1" width="30%">

## Features

- Display GPS data: Latitude, Longitude, Altitude, Speed, Course, Date, Time, HDOP
- Track all satellites ever seen, currently visible, and those used in the fix
- Sky plot of satellites with color-coded status:
  - Green: used in fix
  - Yellow: visible
  - Red: not used
- Optionally show satellite ID and system on the plot

<img src="https://raw.githubusercontent.com/alcor55/Cardputer-GPS-Info/main/6.jpeg" alt="Screenshot 1" width="30%">

- Keyboard control:
  - `[s]` Start/Stop GPS serial
  - `[c]` Configuration menu
  - `[h]` Help menu
  - `[l]` Print satellite list to USB serial
  - `[n]` Print NMEA sentences to USB serial
  - `[p]` Show/hide satellite ID on sky plot
  - `[o]` Show/hide system on sky plot

<img src="https://raw.githubusercontent.com/alcor55/Cardputer-GPS-Info/main/2.jpeg" alt="Screenshot 2" width="30%">
<img src="https://raw.githubusercontent.com/alcor55/Cardputer-GPS-Info/main/3.jpeg" alt="Screenshot 3" width="30%">

## Hardware

- M5Stack Cardputer (1, 1.1, ADV)
- GPS module connected to configurable TX/RX pins (configurables[c] key)

<img src="https://raw.githubusercontent.com/alcor55/Cardputer-GPS-Info/main/4.jpeg" alt="Screenshot 4" width="30%">

## NMEA sentences to USB serial `[n]`

If you wat to share the cardputer gps data via serial to another software now you can!
Press n, with the cardputer connected via usb, open your nmea software, select the port, enjoy!

<img src="https://raw.githubusercontent.com/alcor55/Cardputer-GPS-Info/main/5.jpeg" alt="Screenshot 5" width="50%">

## Print satellite list to USB serial `[l]`

<img src="https://raw.githubusercontent.com/alcor55/Cardputer-GPS-Info/main/7.png" alt="Screenshot 7" width="30%">
