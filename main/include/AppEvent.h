/*
 *
 *    Copyright (c) 2022-2023 Project CHIP Authors
 *    All rights reserved.
 *
 *    Licensed under the Apache License, Version 2.0 (the "License");
 *    you may not use this file except in compliance with the License.
 *    You may obtain a copy of the License at
 *
 *        http://www.apache.org/licenses/LICENSE-2.0
 *
 *    Unless required by applicable law or agreed to in writing, software
 *    distributed under the License is distributed on an "AS IS" BASIS,
 *    WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 *    See the License for the specific language governing permissions and
 *    limitations under the License.
 */

#pragma once

#include <stdbool.h>
#include <stdint.h>

struct AppEvent;
typedef void (*EventHandler)(AppEvent *);

struct AppEvent
{
    enum AppEventTypes
    {
        kEventType_Temperature = 0,
        kEventType_Humidity,
        kEventType_Temp_and_humidity
    };

    uint16_t Type;

    union
    {
        struct
        {
            float Temperature;
        } TempEvent;
        struct
        {
            float Humidity;
        } HumidityEvent;
        struct
        {
            float Temperature;
            float Humidity;
        } TempAndHumidityEvent;
    };

    EventHandler mHandler;
};
