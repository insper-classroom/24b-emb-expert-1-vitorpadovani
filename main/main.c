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


// #include "servo.h"
// #include <stdio.h>
// #include "pico/stdlib.h"
// #include "FreeRTOS.h"
// #include "task.h"


// bool directionOne = true;
// int currentMillisOne = 400;
// int servoPinOne = 2;

// bool directionTwo = false;
// int currentMillisTwo = 1600;
// int servoPinTwo = 3;

// void servo_task_one(void *pvParameters)
// {
//     // Configura o servo inicial
//     setServo(servoPinOne, currentMillisOne);

//     while (true)
//     {
//         // Ajusta o tempo do pulso para servo 1
//         currentMillisOne += (directionOne) ? 5 : -5;
        
//         if (currentMillisOne >= 2400) directionOne = false;
//         if (currentMillisOne <= 400) directionOne = true;

//         setMillis(servoPinOne, currentMillisOne);

//         // Delay de 10 ms
//         vTaskDelay(pdMS_TO_TICKS(10));
//     }
// }

// void servo_task_two(void *pvParameters)
// {
//     // Configura o servo inicial
//     setServo(servoPinTwo, currentMillisTwo);

//     while (true)
//     {
//         // Ajusta o tempo do pulso para servo 2
//         currentMillisTwo += (directionTwo) ? 7 : -7;

//         if (currentMillisTwo >= 2400) directionTwo = false;
//         if (currentMillisTwo <= 400) directionTwo = true;

//         setMillis(servoPinTwo, currentMillisTwo);

//         // Delay de 10 ms
//         vTaskDelay(pdMS_TO_TICKS(10));
//     }
// }

// int main()
// {
//     stdio_init_all();

//     // Cria as tarefas para cada servo
//     xTaskCreate(servo_task_one, "Servo Task One", 256, NULL, 1, NULL);
//     xTaskCreate(servo_task_two, "Servo Task Two", 256, NULL, 1, NULL);

//     // Inicia o scheduler do FreeRTOS
//     vTaskStartScheduler();

//     // Loop infinito caso o scheduler falhe
//     while (true) {}

//     return 0;
// }



// #include "servo.h"
// #include <stdio.h>
// #include "pico/stdlib.h"
// #include "hardware/adc.h"

// // Definição dos pinos dos servos e do joystick
// int servoPinOne = 2;
// int servoPinTwo = 3;
// int joystickX = 26; // Supondo que o eixo X do joystick está no ADC0 (GPIO 26)
// int joystickY = 27; // Supondo que o eixo Y do joystick está no ADC1 (GPIO 27)

// // Variáveis para controlar a posição dos servos
// int currentMillisOne = 1400; // Posição inicial do servo 1 (valor médio)
// int currentMillisTwo = 1400; // Posição inicial do servo 2 (valor médio)
// const int servoMin = 400;
// const int servoMax = 2400;

// void setup() {
//     // Configuração dos pinos dos servos
//     setServo(servoPinOne, currentMillisOne);
//     setServo(servoPinTwo, currentMillisTwo);

//     // Inicialização do ADC para leitura analógica
//     adc_init();
//     adc_gpio_init(joystickX);
//     adc_gpio_init(joystickY);
// }

// int readJoystick(int channel) {
//     adc_select_input(channel);
//     return adc_read();
// }

// int mapValue(int value, int in_min, int in_max, int out_min, int out_max) {
//     return (value - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
// }

// int main() {
//     setup();

//     while (true) {
//         // Leitura dos valores do joystick
//         int xValue = readJoystick(0); // Lê o eixo X do joystick (canal 0)
//         int yValue = readJoystick(1); // Lê o eixo Y do joystick (canal 1)

//         // Mapeamento dos valores do joystick para o intervalo do servo
//         currentMillisOne = mapValue(xValue, 0, 4095, servoMin, servoMax);
//         currentMillisTwo = mapValue(yValue, 0, 4095, servoMin, servoMax);

//         // Definindo a posição dos servos
//         setMillis(servoPinOne, currentMillisOne);
//         setMillis(servoPinTwo, currentMillisTwo);

//         sleep_ms(10); // Pequeno delay para evitar leituras excessivas
//     }
// }


#include "servo.h"
#include <stdio.h>
#include "pico/stdlib.h"
#include "hardware/adc.h"
#include "hardware/uart.h"
#include "FreeRTOS.h"
#include "task.h"

// Definição dos pinos dos servos e do joystick
#define SERVO_PIN_ONE 2
#define SERVO_PIN_TWO 3
#define JOYSTICK_X_PIN 26 // Joystick X no ADC0 (GPIO 26)
#define JOYSTICK_Y_PIN 27 // Joystick Y no ADC1 (GPIO 27)

// Definições para a conversão do joystick
#define MAX_VALUE 4095          // Valor máximo do ADC para 12 bits
#define SCALE_FACTOR 255        // Escala para mapeamento de -255 a +255
#define DEAD_ZONE 100           // Tamanho da zona morta
#define MAX_SERVO_INCREMENT 10  // Incremento máximo por leitura do joystick

// Funções de configuração e utilitárias
void setup() {
    // Configuração dos pinos dos servos com valores iniciais fixos
    setServo(SERVO_PIN_ONE, 1600); // Posição inicial do servo 1
    setServo(SERVO_PIN_TWO, 1600); // Posição inicial do servo 2

    // Inicialização do ADC para leitura analógica
    adc_init();
    adc_gpio_init(JOYSTICK_X_PIN);
    adc_gpio_init(JOYSTICK_Y_PIN);

    // Inicialização da UART para envio de dados do sensor
    uart_init(uart0, 9600);
    gpio_set_function(0, GPIO_FUNC_UART);
    gpio_set_function(1, GPIO_FUNC_UART);
}

int readJoystick(int channel) {
    adc_select_input(channel);
    return adc_read();
}

// Função para converter valor do joystick em movimento (-255 a +255)
int converter_valor(int adc_value) {
    int valor = ((adc_value - (MAX_VALUE / 2)) * SCALE_FACTOR) / (MAX_VALUE / 2);

    if (valor > -DEAD_ZONE && valor < DEAD_ZONE) {
        return 0; // Retorna 0 se o valor estiver dentro da zona morta
    }
    return valor;
}

// Mapeamento de valores
int mapValue(int value, int in_min, int in_max, int out_min, int out_max) {
    if (value < in_min) value = in_min;
    if (value > in_max) value = in_max;
    return (value - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}

// Tarefa FreeRTOS para controle dos servos
void vTaskServoControl(void *pvParameters) {
    // Variáveis locais estáticas para manter o estado entre as iterações
    static int currentMillisOne = 1600; // Posição inicial do servo 1
    static int currentMillisTwo = 1600; // Posição inicial do servo 2

    // Constantes locais
    const int servoMin = 400;
    const int servoMax = 2400;

    while (1) {
        // Leitura dos valores do joystick
        int xValue = readJoystick(0); // Eixo X do joystick (canal 0)
        int yValue = readJoystick(1); // Eixo Y do joystick (canal 1)
        printf("X: %d, Y: %d\n", xValue, yValue);

        // Conversão dos valores do joystick
        int xMovement = converter_valor(xValue);
        int yMovement = converter_valor(yValue);
        printf("X modificado: %d, Y modificado: %d\n", xMovement, yMovement);

        // Ajustando a posição do servo de forma suave
        if (xMovement != 0) {
            currentMillisOne += (xMovement / 255.0) * MAX_SERVO_INCREMENT;
            currentMillisOne = mapValue(currentMillisOne, servoMin, servoMax, servoMin, servoMax);
        }

        if (yMovement != 0) {
            currentMillisTwo += (yMovement / 255.0) * MAX_SERVO_INCREMENT;
            currentMillisTwo = mapValue(currentMillisTwo, servoMin, servoMax, servoMin, servoMax);
        }

        // Definindo a posição dos servos
        setMillis(SERVO_PIN_ONE, currentMillisOne);
        setMillis(SERVO_PIN_TWO, currentMillisTwo);

        vTaskDelay(pdMS_TO_TICKS(10)); // Delay de 10 ms
    }
}

// Tarefa FreeRTOS para leitura do sensor e envio via UART
void vTaskSensorRead(void *pvParameters) {
    while (1) {
        // Leitura do sensor adicional
        adc_select_input(2); // Leitura do sensor no canal 2
        int sensorValue = adc_read();

        // Processamento (aqui simplificado, mas pode incluir filtragem, etc.)
        char sensorData[20];
        snprintf(sensorData, sizeof(sensorData), "Sensor: %d\r\n", sensorValue);

        // Envio dos dados do sensor via UART
        uart_puts(uart0, sensorData);

        vTaskDelay(pdMS_TO_TICKS(1000)); // Delay de 1 segundo
    }
}

int main() {
    setup();


    // Criação das tarefas
    xTaskCreate(vTaskServoControl, "ServoControl", 256, NULL, 1, NULL);
    xTaskCreate(vTaskSensorRead, "SensorRead", 256, NULL, 1, NULL);

    // Início do agendador do FreeRTOS
    vTaskStartScheduler();

    // O código não deve chegar aqui
    while (1) {}
}