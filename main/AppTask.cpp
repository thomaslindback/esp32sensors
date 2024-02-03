#include "AppTask.h"
#include "esp_log.h"
#include "freertos/FreeRTOS.h"
#include "freertos/timers.h"
#include "driver/gpio.h"

#include "ASHT31.h"
#include "SMAX1704x_Fuel_Gauge.h"
#include "wifi_t.h"

#define APP_TASK_NAME "APP"
#define APP_EVENT_QUEUE_SIZE 10
#define APP_TASK_STACK_SIZE (3072)
#define BUTTON_PRESSED 1
#define APP_LIGHT_SWITCH 1
#define GPIO_OUTPUT_IO_0   1
#define GPIO_OUPUT_PIN_SEL  (1ULL<<GPIO_OUTPUT_IO_0)

static const char * TAG = "app-task";

namespace {
QueueHandle_t sAppEventQueue;
TaskHandle_t sAppTaskHandle;
TimerHandle_t sTimer;
} // namespace

AppTask AppTask::sAppTask;
//sht31::ASHT31 sensor;
max1704x::SFE_MAX1704X fuel;

esp_err_t AppTask::sendData(float v, float p) {
    uint16_t MAX_HTTP_OUTPUT_BUFFER = 256;
    char local_response_buffer[MAX_HTTP_OUTPUT_BUFFER] = {0};
    char buffer [50];
    sprintf (buffer, "/hello/%f/%f", v,p);
    esp_http_client_config_t config = {
        .host = "192.168.1.242",
        .path = buffer,
        .disable_auto_redirect = true,
        //.event_handler = _http_event_handler,
        .user_data = local_response_buffer, 
    };
    esp_http_client_handle_t client = esp_http_client_init(&config);

    // GET
    esp_err_t err = esp_http_client_perform(client);
    if (err == ESP_OK) {
        ESP_LOGI(TAG, "HTTP GET Status = %d, content_length = %"PRIu64,
                esp_http_client_get_status_code(client),
                esp_http_client_get_content_length(client));
    } else {
        ESP_LOGE(TAG, "HTTP GET request failed: %s", esp_err_to_name(err));
    }
    return err;
}

esp_err_t AppTask::StartAppTask()
{
    ESP_LOGI(TAG, "I2C SCL = %i", SHT31_I2C_MASTER_SCL);
    ESP_LOGI(TAG, "I2C SDA = %i", SHT31_I2C_MASTER_SDA);

    //if(!sensor.begin()) {
    //    ESP_LOGI(TAG, "Failed to begin sht31");
    //}
    if(!fuel.begin()) {
        ESP_LOGI(TAG, "Failed to begin fuel gauge");
    } 
    sAppEventQueue = xQueueCreate(APP_EVENT_QUEUE_SIZE, sizeof(AppEvent));
    if (sAppEventQueue == NULL)
    {
        ESP_LOGE(TAG, "Failed to allocate app event queue");
        return ESP_FAIL;
    }

    sTimer = xTimerCreate("Timer1", pdMS_TO_TICKS(15000), 
                     pdTRUE, ( void * ) 0, OnTimerTimeout);
     xTimerStart( sTimer, pdMS_TO_TICKS(5000) );

    // Start App task.
    BaseType_t xReturned;
    xReturned = xTaskCreate(AppTaskMain, APP_TASK_NAME, APP_TASK_STACK_SIZE, NULL, 1, &sAppTaskHandle);
    return (xReturned == pdPASS) ? ESP_OK : ESP_FAIL;
}

void AppTask::TestEventHandler(AppEvent * aEvent) {
    ESP_LOGI(TAG, "Voltage: %f", fuel.getVoltage());
    ESP_LOGI(TAG, "Percent: %f", fuel.getSOC());
    ESP_LOGI(TAG, "Change rate: %f", fuel.getChangeRate());

    /*switch (aEvent->Type)
    {
    case AppEvent::kEventType_Temperature:
        ESP_LOGI(TAG, "Temp: %f", aEvent->TempEvent.Temperature);
        ESP_LOGI(TAG, "Bat: %f", fuel.getVoltage());
        ESP_LOGI(TAG, "Temp: %f", fuel.getChangeRate());
        break;
    default:
        break;
    }*/

}

int level = 0;
void AppTask::OnTimerTimeout(TimerHandle_t xTimer)
{
    gpio_set_level(GPIO_NUM_1, level);
    if(level == 0) {
        level = 1;
    } else {
        level = 0;
    }
    AppEvent timer_event = {};
    timer_event.Type     = AppEvent::kEventType_Temperature;
    timer_event.TempEvent.Temperature = 0; //sensor.readTemperature();
    timer_event.mHandler = AppTask::TestEventHandler;

    sAppTask.PostEvent(&timer_event);

    float v = fuel.getVoltage();
    ESP_LOGI(TAG, "Voltage: %f", v);
    float p = fuel.getSOC();
    ESP_LOGI(TAG, "Percent: %f", p);
    //ESP_LOGI(TAG, "Change rate: %f", fuel.getChangeRate());
    sendData(v,p);

}

esp_err_t AppTask::Init()
{
    gpio_config_t io_conf = {};
    //disable interrupt
    io_conf.intr_type = GPIO_INTR_DISABLE;
    //set as output mode
    io_conf.mode = GPIO_MODE_OUTPUT;
    //bit mask of the pins that you want to set,e.g.GPIO18/19
    io_conf.pin_bit_mask = GPIO_OUPUT_PIN_SEL;
    //disable pull-down mode
    io_conf.pull_down_en = GPIO_PULLDOWN_DISABLE;
    //disable pull-up mode
    io_conf.pull_up_en = GPIO_PULLUP_DISABLE;
    //configure GPIO with the given settings
    gpio_config(&io_conf);
    gpio_set_level(GPIO_NUM_1, 1);

    ESP_LOGI(TAG, "ESP_WIFI_MODE_STA");
    wifi_init_sta();
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

