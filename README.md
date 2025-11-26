# ESP32-C3 GPS NMEA → UDP Bridge

ESP32-C3 firmware that reads NMEA data from u-blox GPS receiver and forwards it via UDP.

## Features

- **Access Point Mode**: Creates WiFi AP for configuration (SSID: `GPS_ESP32`)
- **Station Mode**: Connects to existing WiFi network
- **Web Interface**: Configure WiFi, UDP target, and GPS settings via browser
- **UDP Forwarding**: Sends NMEA sentences via UDP (unicast or broadcast)
- **Rate Limiting**: UDP transmission limited to 1 second intervals
- **UBX Configuration**: Configure u-blox GPS dynamic model and update rate

## Hardware

- ESP32-C3 microcontroller
- u-blox GPS module (UART connection)
- Default pins: RX=20, TX=21

## Configuration

1. Connect to WiFi AP `GPS_ESP32` (password: `12345678`)
2. Open browser to `192.168.4.1`
3. Configure:
   - WiFi credentials for station mode
   - UDP target IP and port
   - GPS dynamic model (Portable/Automotive/Sea/etc)
   - Navigation update rate (1-10 Hz)

## UDP Output

NMEA sentences are forwarded via UDP with:
- 1 second rate limiting
- CRLF line endings
- Optional broadcast mode

## License

See LICENSE file.