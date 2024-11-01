/**
 * Copyright (c) 2020 Raspberry Pi (Trading) Ltd.
 *
 * SPDX-License-Identifier: BSD-3-Clause
 */

// #include "servo.h"
// #include <stdio.h>
// #include "pico/stdlib.h"

// bool directionOne = true;
// int currentMillisOne = 400;
// int servoPinOne = 2;

// bool directionTwo = false;
// int currentMillisTwo = 1600;
// int servoPinTwo = 3;

// int main()
// {
//     setServo(servoPinOne, currentMillisOne);
//     setServo(servoPinTwo, currentMillisTwo);
//     while (true)
//     {
//         currentMillisOne += (directionOne)?5:-5;
//         if (currentMillisOne >= 2400) directionOne = false;
//         if (currentMillisOne <= 400) directionOne = true;
//         setMillis(servoPinOne, currentMillisOne);

//         currentMillisTwo += (directionTwo)?7:-7;
//         if (currentMillisTwo >= 2400) directionTwo = false;
//         if (currentMillisTwo <= 400) directionTwo = true;
//         setMillis(servoPinTwo, currentMillisTwo);

//         sleep_ms(10);
//     }
// }


#include "servo.h"
#include <stdio.h>
#include "pico/stdlib.h"
#include "FreeRTOS.h"
#include "task.h"


bool directionOne = true;
int currentMillisOne = 400;
int servoPinOne = 2;

bool directionTwo = false;
int currentMillisTwo = 1600;
int servoPinTwo = 3;

void servo_task_one(void *pvParameters)
{
    // Configura o servo inicial
    setServo(servoPinOne, currentMillisOne);

    while (true)
    {
        // Ajusta o tempo do pulso para servo 1
        currentMillisOne += (directionOne) ? 5 : -5;
        
        if (currentMillisOne >= 2400) directionOne = false;
        if (currentMillisOne <= 400) directionOne = true;

        setMillis(servoPinOne, currentMillisOne);

        // Delay de 10 ms
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

void servo_task_two(void *pvParameters)
{
    // Configura o servo inicial
    setServo(servoPinTwo, currentMillisTwo);

    while (true)
    {
        // Ajusta o tempo do pulso para servo 2
        currentMillisTwo += (directionTwo) ? 7 : -7;

        if (currentMillisTwo >= 2400) directionTwo = false;
        if (currentMillisTwo <= 400) directionTwo = true;

        setMillis(servoPinTwo, currentMillisTwo);

        // Delay de 10 ms
        vTaskDelay(pdMS_TO_TICKS(10));
    }
}

int main()
{
    stdio_init_all();

    // Cria as tarefas para cada servo
    xTaskCreate(servo_task_one, "Servo Task One", 256, NULL, 1, NULL);
    xTaskCreate(servo_task_two, "Servo Task Two", 256, NULL, 1, NULL);

    // Inicia o scheduler do FreeRTOS
    vTaskStartScheduler();

    // Loop infinito caso o scheduler falhe
    while (true) {}

    return 0;
}

