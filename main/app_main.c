/////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////// DEFINITION DES BIBLIOTHEQUES
/////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////
#include <stdio.h>

#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/semphr.h"
#include "freertos/task.h"
#include "gps.h"             // Interface pour les fonctionnalités GPS
#include "initialisation.h"  // Interface pour la gestion de l'écran tactile

/////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////
///////////////////////// DEFINITION DES FONCTIONS
/////////////////////////////////////////////////////////////////////////////////////////////////////
/////////////////////////////////////////////////////////////////////////////////////////////////////

void app_main(void) {
  // Exécute l'initialisation du système
  system_initialization_task();

  // Ajoute une petite pause avant de lancer les autres tâches
  vTaskDelay(pdMS_TO_TICKS(100));  // Délai de 100 ms

  // // Lancer les autres tâches après l'initialisation
  xTaskCreate(gps_task, "GPS_Task", 4096, NULL, 1, NULL);
}