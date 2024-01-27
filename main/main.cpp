/* FreeRTOS Real Time Stats Example

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/

#include <stdio.h>
#include <stdlib.h>
#include <inttypes.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "freertos/semphr.h"
#include "esp_err.h"
#include "esp_log.h"

#include "AppTask.h"

static const char * TAG = "teel-plug-t1-app";

extern "C" void app_main(void)
{
    ESP_LOGI(TAG, "Starting");
    esp_err_t error = GetAppTask().StartAppTask();
    if (error != ESP_OK)
    {
        ESP_LOGE(TAG, "GetAppTask().StartAppTask() failed : %i", error);
    }

}
