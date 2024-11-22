#include "gps.h"

#include "initialisation.h"

/////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////// DEFINITION DES CONSTANTES
/////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////
#define GPS_TAG "GPS_Module"
#define GPS_BUFFER_SIZE 1024

/////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////// DEFINITION DES VARIABLES STATIQUES
/////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////
static char gps_buffer[GPS_BUFFER_SIZE];
static int gps_buffer_pos = 0;

/////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////// DEFINITION DES FONCTIONS
/////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////

/**
 * @brief Internal function to parse $GPGGA NMEA frames and extract data
 *
 * @param nmea: NMEA string containing the $GPGGA frame
 * @param gps_data: Pointer to GPSData_t structure to store extracted data
 *
 * @return
 *      - true if parsing is successful, false otherwise
 */
static bool parse_gpgga(const char *nmea, GPSData_t *gps_data) {
  // ESP_LOGI(GPS_TAG, "Parsing $GPGGA frame");
  float lat, lon;
  char ns, ew;
  int quality, satellites;
  float altitude;

  int parsed = sscanf(nmea, "$GPGGA,%*f,%f,%c,%f,%c,%d,%d,%*f,%f",
                      &lat, &ns, &lon, &ew, &quality, &satellites, &altitude);

  if (parsed < 7) {
    ESP_LOGE(GPS_TAG, "Failed to parse $GPGGA frame");
    return false;
  }

  // Conversion en coordonnées décimales
  float lat_deg = (int)(lat / 100);
  float lat_min = lat - (lat_deg * 100);
  gps_data->latitude = lat_deg + lat_min / 60.0f;
  if (ns == 'S') gps_data->latitude = -gps_data->latitude;

  float lon_deg = (int)(lon / 100);
  float lon_min = lon - (lon_deg * 100);
  gps_data->longitude = lon_deg + lon_min / 60.0f;
  if (ew == 'W') gps_data->longitude = -gps_data->longitude;

  gps_data->altitude = altitude;

  // ESP_LOGI(GPS_TAG, "Parsed $GPGGA: Lat: %.6f, Lon: %.6f, Altitude: %.2f",
  //         gps_data->latitude, gps_data->longitude, gps_data->altitude);

  return true;
}

/**
 * @brief Internal function to parse $GPRMC NMEA frames and extract data
 *
 * @param nmea: NMEA string containing the $GPRMC frame
 * @param gps_data: Pointer to GPSData_t structure to store extracted data
 *
 * @return
 *      - true if parsing is successful, false otherwise
 */
static bool parse_gprmc(const char *nmea, GPSData_t *gps_data) {
  // ESP_LOGI(GPS_TAG, "Parsing $GPRMC frame");

  // Copier la trame NMEA pour la manipulation
  char nmea_copy[100];
  strncpy(nmea_copy, nmea, sizeof(nmea_copy) - 1);
  nmea_copy[sizeof(nmea_copy) - 1] = '\0';  // Assurez-vous que la chaîne est terminée

  char *token;
  char *rest = nmea_copy;
  int field_index = 0;

  // Réinitialiser les données GPS
  gps_data->valid = false;

  while ((token = strtok_r(rest, ",", &rest))) {
    field_index++;
    switch (field_index) {
      case 2:  // Heure (hhmmss.ss)
        if (strlen(token) >= 6) {
          char hour_str[3] = {token[0], token[1], '\0'};
          char min_str[3] = {token[2], token[3], '\0'};
          char sec_str[3] = {token[4], token[5], '\0'};
          gps_data->hour = atoi(hour_str);
          gps_data->minute = atoi(min_str);
          gps_data->second = atoi(sec_str);
        } else {
          ESP_LOGE(GPS_TAG, "Invalid time format in $GPRMC sentence");
          return false;
        }
        break;
      case 3:  // Statut (A=Actif, V=Void)
        gps_data->valid = (token[0] == 'A');
        if (!gps_data->valid) {
          ESP_LOGE(GPS_TAG, "GPRMC status is invalid");
          return false;
        }
        break;
      case 4:  // Latitude
        if (strlen(token) > 0) {
          gps_data->latitude = atof(token);
        } else {
          ESP_LOGE(GPS_TAG, "Invalid latitude data in $GPRMC sentence");
          return false;
        }
        break;
      case 5:  // N/S
        if (token[0] == 'S') gps_data->latitude = -gps_data->latitude;
        break;
      case 6:  // Longitude
        if (strlen(token) > 0) {
          gps_data->longitude = atof(token);
        } else {
          ESP_LOGE(GPS_TAG, "Invalid longitude data in $GPRMC sentence");
          return false;
        }
        break;
      case 7:  // E/W
        if (token[0] == 'W') gps_data->longitude = -gps_data->longitude;
        break;
      case 8:  // Vitesse
        gps_data->speed = atof(token);
        break;
      case 9:  // Date (ddmmyy)
        if (strlen(token) == 6) {
          char day_str[3] = {token[0], token[1], '\0'};
          char month_str[3] = {token[2], token[3], '\0'};
          char year_str[3] = {token[4], token[5], '\0'};
          gps_data->day = atoi(day_str);
          gps_data->month = atoi(month_str);
          gps_data->year = 2000 + atoi(year_str);  // Ajoute 2000 pour obtenir l'année complète
        } else {
          ESP_LOGE(GPS_TAG, "Invalid date format in $GPRMC sentence");
          return false;
        }
        break;
      default:
        break;
    }
  }

  // Validation et conversion finale
  if (gps_data->valid) {
    float lat_deg = (int)(gps_data->latitude / 100);
    float lat_min = gps_data->latitude - (lat_deg * 100);
    gps_data->latitude = lat_deg + lat_min / 60.0f;

    float lon_deg = (int)(gps_data->longitude / 100);
    float lon_min = gps_data->longitude - (lon_deg * 100);
    gps_data->longitude = lon_deg + lon_min / 60.0f;

    // ESP_LOGI(GPS_TAG, "Parsed $GPRMC: Lat: %.6f, Lon: %.6f, Speed: %.2f, Date: %02d/%02d/%04d, Time: %02d:%02d:%02d",
    //          gps_data->latitude, gps_data->longitude, gps_data->speed,
    //          gps_data->day, gps_data->month, gps_data->year,
    //          gps_data->hour, gps_data->minute, gps_data->second);
  } else {
    ESP_LOGE(GPS_TAG, "Parsed data is invalid");
  }

  return gps_data->valid;
}

/**
 * @brief Verify the checksum of an NMEA string
 *
 * @param nmea: NMEA string to verify
 *
 * @return
 *      - true if checksum matches, false otherwise
 */
static bool verify_nmea_checksum(const char *nmea) {
  // ESP_LOGI(GPS_TAG, "Verifying NMEA checksum");
  const char *checksum_position = strchr(nmea, '*');
  if (checksum_position == NULL) {
    ESP_LOGE(GPS_TAG, "Invalid NMEA frame: No checksum found");
    return false;
  }

  uint8_t checksum = 0;
  for (const char *p = nmea + 1; p < checksum_position; ++p) {
    checksum ^= *p;
  }

  uint8_t provided_checksum = (uint8_t)strtol(checksum_position + 1, NULL, 16);
  if (checksum == provided_checksum) {
    // ESP_LOGI(GPS_TAG, "NMEA checksum is valid");
    return true;
  } else {
    ESP_LOGE(GPS_TAG, "NMEA checksum mismatch: Calculated: 0x%X, Provided: 0x%X", checksum, provided_checksum);
    return false;
  }
}

/**
 * @brief Initialize the UART for GPS communication
 *
 * @param baud_rate: Baud rate for the UART communication
 * @param tx_pin: GPIO pin number for UART TX
 * @param rx_pin: GPIO pin number for UART RX
 *
 * @return
 *      - ESP_OK on success
 *      - ESP_ERR_* code on failure
 */
esp_err_t GPS_Init(uint32_t baud_rate, int tx_pin, int rx_pin) {
  ESP_LOGI(GPS_TAG, "Initializing GPS UART with baud rate %" PRIu32 ", TX pin %d, RX pin %d", baud_rate, tx_pin, rx_pin);

  // Configuration des paramètres UART
  const uart_config_t uart_config = {
      .baud_rate = baud_rate,
      .data_bits = UART_DATA_8_BITS,
      .parity = UART_PARITY_DISABLE,
      .stop_bits = UART_STOP_BITS_1,
      .flow_ctrl = UART_HW_FLOWCTRL_DISABLE};

  esp_err_t ret = uart_param_config(UART_NUM, &uart_config);
  if (ret != ESP_OK) {
    ESP_LOGE(GPS_TAG, "UART parameter configuration failed: %s", esp_err_to_name(ret));
    return ESP_ERR_GPS_UART_PARAM_CONFIG_FAILED;
  }

  // Configuration des broches UART
  ret = uart_set_pin(UART_NUM, tx_pin, rx_pin, UART_PIN_NO_CHANGE, UART_PIN_NO_CHANGE);
  if (ret != ESP_OK) {
    ESP_LOGE(GPS_TAG, "UART set pin failed: %s", esp_err_to_name(ret));
    return ESP_ERR_GPS_UART_SET_PIN_FAILED;
  }

  // Installation du pilote UART
  ret = uart_driver_install(UART_NUM, BUF_SIZE * 2, 0, 0, NULL, 0);
  if (ret != ESP_OK) {
    ESP_LOGE(GPS_TAG, "UART driver installation failed: %s", esp_err_to_name(ret));
    return ESP_ERR_GPS_UART_DRIVER_INSTALL_FAILED;
  }

  ESP_LOGI(GPS_TAG, "GPS UART initialized successfully");
  return ESP_OK;
}

/**
 * @brief Read and process NMEA frames from GPS module
 *
 * @param gps_data: Pointer to GPSData_t structure to store parsed data
 *
 * @return
 *      - true if valid GPS data is read, false otherwise
 */
bool GPS_ReadData(GPSData_t *gps_data) {
  uint8_t data[BUF_SIZE];
  int len = uart_read_bytes(UART_NUM, data, BUF_SIZE - 1, 20 / portTICK_PERIOD_MS);
  if (len > 0) {
    // Accumuler les données dans gps_buffer
    if (gps_buffer_pos + len >= GPS_BUFFER_SIZE) {
      // Débordement du buffer, réinitialiser
      ESP_LOGE(GPS_TAG, "GPS buffer overflow");
      gps_buffer_pos = 0;
      gps_data->valid = false;
      return false;
    }
    memcpy(gps_buffer + gps_buffer_pos, data, len);
    gps_buffer_pos += len;
    gps_buffer[gps_buffer_pos] = '\0';  // Terminer la chaîne

    // Traiter les lignes complètes
    char *line_start = gps_buffer;
    char *newline;
    while ((newline = strchr(line_start, '\n')) != NULL) {
      *newline = '\0';
      if (newline > line_start && *(newline - 1) == '\r') {
        *(newline - 1) = '\0';
      }

      // Traiter la ligne
      if (strstr(line_start, "$GPGGA") != NULL) {
        // ESP_LOGI(GPS_TAG, "Processing $GPGGA frame: %s", line_start);
        if (verify_nmea_checksum(line_start) && parse_gpgga(line_start, gps_data)) {
          gps_data->valid = true;
          // ESP_LOGI(GPS_TAG, "$GPGGA frame parsed successfully");
        }
      } else if (strstr(line_start, "$GPRMC") != NULL) {
        // ESP_LOGI(GPS_TAG, "Processing $GPRMC frame: %s", line_start);
        if (verify_nmea_checksum(line_start) && parse_gprmc(line_start, gps_data)) {
          gps_data->valid = true;
          // ESP_LOGI(GPS_TAG, "$GPRMC frame parsed successfully");
        }
      }

      line_start = newline + 1;
    }

    // Déplacer les données restantes (incomplètes)
    int remaining = gps_buffer + gps_buffer_pos - line_start;
    memmove(gps_buffer, line_start, remaining);
    gps_buffer_pos = remaining;
    gps_buffer[gps_buffer_pos] = '\0';

    return gps_data->valid;
  }
  ESP_LOGW(GPS_TAG, "No valid GPS data received");
  gps_data->valid = false;
  return false;
}

/**
 * @brief GPS Task for independent GPS processing
 */
void gps_task(void *param) {
  GPSData_t gps_data;

  while (1) {
    if (GPS_ReadData(&gps_data)) {
      if (gps_data.valid) {
        /// affichage des informations
        ESP_LOGI("GPS_Task", "Position: Latitude %.6f, Longitude %.6f", gps_data.latitude, gps_data.longitude);
        ESP_LOGI("GPS_Task", "Date: %02d/%02d/%04d", gps_data.day, gps_data.month, gps_data.year);
        ESP_LOGI("GPS_Task", "Heure: %02d:%02d:%02d", gps_data.hour, gps_data.minute, gps_data.second);
        ESP_LOGI("GPS_Task", "Altitude: %.2f m, Vitesse: %.2f noeuds", gps_data.altitude, gps_data.speed);
      } else {
        // xEventGroupClearBits(init_event_group, SYSTEM_STATE_GPS_AVAILABLE);
        ESP_LOGI("GPS_Task", "Données GPS non valides.");
      }
    } else {
      // xEventGroupClearBits(init_event_group, SYSTEM_STATE_GPS_AVAILABLE);
      ESP_LOGI("GPS_Task", "Aucune donnée GPS reçue.");
    }
    vTaskDelay(1000 / portTICK_PERIOD_MS);  // Pause de 1 seconde entre chaque lecture
  }
}

/**
 * @brief Configure le taux de rafraîchissement du module GPS
 *
 * @param rate_hz: Taux de rafraîchissement désiré en Hertz (1 à 10 Hz pour le NEO-M6)
 *
 * @return
 *      - ESP_OK si la configuration est réussie
 *      - ESP_ERR_INVALID_ARG si le taux est hors limites
 */
esp_err_t GPS_SetUpdateRate(uint8_t rate_hz) {
  if (rate_hz < 1 || rate_hz > 10) {
    ESP_LOGE(GPS_TAG, "Taux de rafraîchissement invalide: %d Hz", rate_hz);
    return ESP_ERR_INVALID_ARG;
  }

  // Le taux de rafraîchissement est défini par l'intervalle en millisecondes entre chaque trame
  uint16_t interval_ms = 1000 / rate_hz;

  // Commande UBX-CFG-RATE pour configurer le taux de rafraîchissement
  uint8_t ubx_cmd[] = {
      0xB5, 0x62,  // En-tête UBX
      0x06, 0x08,  // Classe et ID (CFG-RATE)
      0x06, 0x00,  // Taille du payload (6 octets)
      0x00, 0x00,  // MeasRate (intervalle de mesure en ms)
      0x01, 0x00,  // NavRate (taux de navigation, fixe à 1)
      0x01, 0x00,  // TimeRef (0=UTC, 1=GPS)
      0x00, 0x00   // Checksum (CK_A, CK_B)
  };

  // Insérer l'intervalle de mesure
  ubx_cmd[6] = interval_ms & 0xFF;
  ubx_cmd[7] = (interval_ms >> 8) & 0xFF;

  // Calcul du checksum
  uint8_t ck_a = 0, ck_b = 0;
  for (int i = 2; i < 12; i++) {
    ck_a += ubx_cmd[i];
    ck_b += ck_a;
  }
  ubx_cmd[12] = ck_a;
  ubx_cmd[13] = ck_b;

  // Envoyer la commande au module GPS
  int len = uart_write_bytes(UART_NUM, (const char *)ubx_cmd, sizeof(ubx_cmd));
  if (len != sizeof(ubx_cmd)) {
    ESP_LOGE(GPS_TAG, "Échec de l'envoi de la commande UBX-CFG-RATE");
    return ESP_FAIL;
  }

  ESP_LOGI(GPS_TAG, "Taux de rafraîchissement configuré à %d Hz", rate_hz);
  return ESP_OK;
}