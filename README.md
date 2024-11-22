
# GPS NEO-M6 Library for ESP-IDF

## Description

This library provides an interface for using the **NEO-M6 GPS module** with the ESP-IDF framework. It supports parsing NMEA frames, extracting GPS data such as latitude, longitude, altitude, speed, and date/time, and managing GPS configuration, including setting the update rate.

The library uses **UART communication** to receive GPS data and includes checksum verification for data integrity. It supports the `$GPGGA` and `$GPRMC` NMEA frames for precise GPS positioning and navigation data.

## Features

- Parse `$GPGGA` frames for:
  - Latitude, longitude (in decimal degrees)
  - Altitude
  - Number of satellites in view
  - Fix quality
- Parse `$GPRMC` frames for:
  - Latitude, longitude (in decimal degrees)
  - Speed (in knots)
  - Date and time
  - Fix validity
- Verify checksum for NMEA strings.
- Configure GPS update rate (1-10 Hz).
- Example FreeRTOS task to process GPS data.

## Requirements

- **ESP-IDF** framework version `>= v4.0`.
- **NEO-M6 GPS module** (or similar with NMEA support).
- UART GPIO pins configured for ESP32.

## Installation

1. Clone the repository into your ESP-IDF project's components folder:
   ```bash
   git clone https://github.com/your-repository-url components/gps
   ```
2. Include the header file in your project:
   ```c
   #include "gps.h"
   ```

## Quick Start

### 1. Initialize GPS Module

Configure the UART interface and set the desired baud rate:

```c
#include "gps.h"

#define GPS_UART_BAUD 9600
#define TX_PIN 17
#define RX_PIN 16

void app_main() {
    if (GPS_Init(GPS_UART_BAUD, TX_PIN, RX_PIN) == ESP_OK) {
        ESP_LOGI("MAIN", "GPS initialized successfully!");
    } else {
        ESP_LOGE("MAIN", "Failed to initialize GPS.");
    }
}
```

### 2. Read GPS Data

Read and process GPS data in a FreeRTOS task:

```c
void gps_task(void *param) {
    GPSData_t gps_data;

    while (1) {
        if (GPS_ReadData(&gps_data)) {
            if (gps_data.valid) {
                ESP_LOGI("GPS_Task", "Latitude: %.6f, Longitude: %.6f", gps_data.latitude, gps_data.longitude);
                ESP_LOGI("GPS_Task", "Date: %02d/%02d/%04d, Time: %02d:%02d:%02d",
                         gps_data.day, gps_data.month, gps_data.year,
                         gps_data.hour, gps_data.minute, gps_data.second);
                ESP_LOGI("GPS_Task", "Altitude: %.2f m, Speed: %.2f knots",
                         gps_data.altitude, gps_data.speed);
            } else {
                ESP_LOGW("GPS_Task", "Invalid GPS data received.");
            }
        }
        vTaskDelay(pdMS_TO_TICKS(1000));
    }
}
```

### 3. Configure GPS Update Rate

Set the update rate to 5 Hz:

```c
if (GPS_SetUpdateRate(5) == ESP_OK) {
    ESP_LOGI("MAIN", "GPS update rate set to 5 Hz");
} else {
    ESP_LOGE("MAIN", "Failed to set GPS update rate");
}
```

## API Reference

### `esp_err_t GPS_Init(uint32_t baud_rate, int tx_pin, int rx_pin)`

Initializes the UART for communication with the GPS module.

- **baud_rate**: GPS UART baud rate (e.g., 9600).
- **tx_pin**: GPIO number for UART TX.
- **rx_pin**: GPIO number for UART RX.
- **Returns**:
  - `ESP_OK` on success.
  - `ESP_ERR_*` on failure.

---

### `bool GPS_ReadData(GPSData_t *gps_data)`

Reads and parses NMEA frames from the GPS module.

- **gps_data**: Pointer to a `GPSData_t` structure to store parsed data.
- **Returns**:
  - `true` if valid data is received.
  - `false` otherwise.

---

### `esp_err_t GPS_SetUpdateRate(uint8_t rate_hz)`

Configures the GPS module's update rate (1-10 Hz).

- **rate_hz**: Desired update rate in Hz.
- **Returns**:
  - `ESP_OK` if successful.
  - `ESP_ERR_INVALID_ARG` if the rate is out of range.

---

## GPSData_t Structure

```c
typedef struct {
    float latitude;
    float longitude;
    float altitude;
    float speed;
    int day, month, year;
    int hour, minute, second;
    bool valid;
} GPSData_t;
```

## Example Project

Clone the [example project](https://github.com/your-repository-url) to see the library in action with a full FreeRTOS task implementation.

## License

This project is licensed under the MIT License. See the `LICENSE` file for details.

## Contributions

Contributions are welcome! Feel free to submit pull requests or open issues for bugs and feature requests.

