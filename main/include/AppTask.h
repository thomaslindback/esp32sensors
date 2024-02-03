
#pragma once

#include <inttypes.h>

#include "AppEvent.h"
#include "freertos/FreeRTOS.h"
#include "freertos/timers.h"
#include "esp_err.h"

class AppTask
{

public:
    esp_err_t StartAppTask();
    static void AppTaskMain(void * pvParameter);
    static void OnTimerTimeout(TimerHandle_t xTimer);
    void PostEvent(const AppEvent * event);

    void UpdateClusterState();

private:
    friend AppTask & GetAppTask(void);
    esp_err_t Init();
    void DispatchEvent(AppEvent * event);
    static void TestEventHandler(AppEvent * aEvent);
    static esp_err_t sendData(float v, float p);

    static AppTask sAppTask;
};

inline AppTask & GetAppTask(void)
{
    return AppTask::sAppTask;
}
