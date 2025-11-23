#pragma once
/*
 * Application Tasks - Header
 * Purpose: Declarations to initialize and configure FreeRTOS tasks/queues.
 * Notes: Add task prototypes and the init entry point here.
 */

#include "esp_err.h"

/* Entry point to create all application tasks. */
esp_err_t app_tasks_init(void);
