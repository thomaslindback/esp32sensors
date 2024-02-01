#include "AppTask.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/timers.h"

#include "ASHT31.h"
#include "SMAX1704x_Fuel_Gauge.h"

#define APP_TASK_NAME "APP"
#define APP_EVENT_QUEUE_SIZE 10
#define APP_TASK_STACK_SIZE (3072)
#define BUTTON_PRESSED 1
#define APP_LIGHT_SWITCH 1

static const char * TAG = "app-task";

namespace {
QueueHandle_t sAppEventQueue;
TaskHandle_t sAppTaskHandle;
TimerHandle_t sTimer;
} // namespace

AppTask AppTask::sAppTask;
sht31::ASHT31 sensor;
SFE_MAX1704X fuel;

esp_err_t AppTask::StartAppTask()
{
    ESP_LOGI(TAG, "I2C SCL = %i", SHT31_I2C_MASTER_SCL);
    ESP_LOGI(TAG, "I2C SDA = %i", SHT31_I2C_MASTER_SDA);

    if(!sensor.begin()) {
        ESP_LOGI(TAG, "Failed to begin sht31");
    }
    if(!fuel.begin()) {
        ESP_LOGI(TAG, "Failed to begin fuel gauge");
    } 
    sAppEventQueue = xQueueCreate(APP_EVENT_QUEUE_SIZE, sizeof(AppEvent));
    if (sAppEventQueue == NULL)
    {
        ESP_LOGE(TAG, "Failed to allocate app event queue");
        return ESP_FAIL;
    }

    sTimer = xTimerCreate("Timer1", pdMS_TO_TICKS(30000), 
                     pdTRUE, ( void * ) 0, OnTimerEvent
                   );
     xTimerStart( sTimer, pdMS_TO_TICKS(10000) );

    // Start App task.
    BaseType_t xReturned;
    xReturned = xTaskCreate(AppTaskMain, APP_TASK_NAME, APP_TASK_STACK_SIZE, NULL, 1, &sAppTaskHandle);
    return (xReturned == pdPASS) ? ESP_OK : ESP_FAIL;
}

void AppTask::TestEventHandler(AppEvent * aEvent) {
    switch (aEvent->Type)
    {
    case AppEvent::kEventType_Temperature:
        ESP_LOGI(TAG, "Temp: %f", aEvent->TempEvent.Temperature);
        ESP_LOGI(TAG, "Bat: %f", fuel.getVoltage());
        ESP_LOGI(TAG, "Temp: %f", fuel.getChangeRate());
        break;
    default:
        break;
    }

}

void AppTask::OnTimerEvent(TimerHandle_t xTimer)
{
    AppEvent timer_event = {};
    timer_event.Type     = AppEvent::kEventType_Temperature;
    timer_event.TempEvent.Temperature = sensor.readTemperature();
    timer_event.mHandler = AppTask::TestEventHandler;

    sAppTask.PostEvent(&timer_event);
}

esp_err_t AppTask::Init()
{

    return ESP_OK;
}

void AppTask::AppTaskMain(void * pvParameter)
{
    AppEvent event;
    ESP_ERROR_CHECK_WITHOUT_ABORT(sAppTask.Init());
    ESP_LOGI(TAG, "App Task started");

    while (true)
    {
        BaseType_t eventReceived = xQueueReceive(sAppEventQueue, &event, pdMS_TO_TICKS(10));
        while (eventReceived == pdTRUE)
        {
            sAppTask.DispatchEvent(&event);
            eventReceived = xQueueReceive(sAppEventQueue, &event, 0); // return immediately if the queue is empty
        }
    }
}

void AppTask::PostEvent(const AppEvent * aEvent)
{
    if (sAppEventQueue != NULL)
    {
        BaseType_t status;
        if (xPortInIsrContext())
        {
            BaseType_t higherPrioTaskWoken = pdFALSE;
            status                         = xQueueSendFromISR(sAppEventQueue, aEvent, &higherPrioTaskWoken);
        }
        else
        {
            status = xQueueSend(sAppEventQueue, aEvent, 1);
        }
        if (!status)
            ESP_LOGE(TAG, "Failed to post event to app task event queue");
    }
    else
    {
        ESP_LOGE(TAG, "Event Queue is NULL should never happen");
    }
}

void AppTask::DispatchEvent(AppEvent * aEvent)
{
    if (aEvent->mHandler)
    {
        aEvent->mHandler(aEvent);
    }
    else
    {
        ESP_LOGI(TAG, "Event received with no handler. Dropping event.");
    }
}

