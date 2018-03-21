#include <Adafruit_Sensor.h>

/*
Arduino-MAX30100 oximetry / heart rate integrated sensor library
Copyright (C) 2016  OXullo Intersecans <x@brainrapers.org>

This program is free software: you can redistribute it and/or modify
it under the terms of the GNU General Public License as published by
the Free Software Foundation, either version 3 of the License, or
(at your option) any later version.

This program is distributed in the hope that it will be useful,
but WITHOUT ANY WARRANTY; without even the implied warranty of
MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
GNU General Public License for more details.

You should have received a copy of the GNU General Public License
along with this program.  If not, see <http://www.gnu.org/licenses/>.
*/

#include <Wire.h>
#include "MAX30100.h"
#include "MAX30100_PulseOximeter.h"
#include "freertos/event_groups.h"
#include "caracteres.h"
//#include <DHT.h>

#define REPORTING_PERIOD_MS     1200

/* define event bits */
#define TASK_1_BIT        ( 1 << 0 ) //1
#define PIN1_RED1 GPIO_NUM_32
#define PIN1_RED2 GPIO_NUM_33
#define PIN2_RED1 GPIO_NUM_15
#define PIN2_RED2 GPIO_NUM_23
#define PIN3_RED1 GPIO_NUM_27
#define PIN3_RED2 GPIO_NUM_14
#define PIN4_RED1 GPIO_NUM_12
#define PIN4_RED2 GPIO_NUM_13
#define PIN_BLK GPIO_NUM_5
#define PIN_RED_STR GPIO_NUM_18
#define PIN_RED_CLK GPIO_NUM_19
#define PIN_GRN_STR GPIO_NUM_17
#define PIN_GRN_CLK GPIO_NUM_16
#define PIN_DHT11 GPIO_NUM_26
#define SDA_PIN GPIO_NUM_21
#define SCL_PIN GPIO_NUM_22
#define PIN_TCH GPIO_NUM_4

#define DHTTYPE DHT11

#define Mat_Alt 8
#define Mat_Larg 32
#define SDELAY 1
#define T IT_2
#define DELAY_INFO 5000
#define DELAY_BANNER 3000
#define TXT_DEBUG 0
#define CaracteresArray CaracteresArray2


uint32_t Matriz[]={
0b00000000000000000000000000000000,
0b00000000000000000000000000000000,
0b00000000000000000000000000000000,
0b00000000000000000000000000000000,
0b00000000000000000000000000000000,
0b00000000000000000000000000000000,
0b00000000000000000000000000000000,
0b00000000000000000000000000000000
};

uint32_t MatrizROSARIO[]={
0b00000000000000000000000000000000,
0b11100011000110001100111001001100,
0b10010100101000010010100101010010,
0b11100100100110011110111001010010,
0b10100100100001010010101001010010,
0b10010011000110010010100101001100,
0b00000000000000000000000000000000,
0b00000000000000000000000000000000
};
uint32_t MatrizSMART[]={
0b00000000000000000000000000000000,
0b00000110011011001100111001110000,
0b01101000010101010010100100100000,
0b00000110010001011110111000100000,
0b01100001010001010010101000100000,
0b00000110010001010010100100100000,
0b00000000000000000000000000000000,
0b00000000000000000000000000000000
};
uint32_t MatrizCIOR[]={
0b00000000000000000000000000000000,
0b00000000110010011000111000000000,
0b00111001001010100100100100111000,
0b00000001000010100100111000000000,
0b00111001001010100100101000111000,
0b00000000110010011000100100000000,
0b00000000000000000000000000000000,
0b00000000000000000000000000000000
};

// PulseOximeter is the higher level interface to the sensor
// it offers:
//  * beat detection reporting
//  * heart rate calculation
//  * SpO2 (oxidation level) calculation
MAX30100 sensor;
PulseOximeter ox;

uint32_t tsLastReport = 0;
int thresholdt = 50;
int thresholdu = 51;
int tread0=100;
int tch=0;
int tchv=0;
static int LattaskCore = 0;
/* create event group */
EventGroupHandle_t eg;
EventGroupHandle_t eglat;
//creo el manejador para el semáforo como variable global
//SemaphoreHandle_t xSemaphore = NULL;
EventBits_t xbit0;
EventBits_t xbit1;
EventBits_t xbit2;


void binary(uint32_t numero){
        for (int i = 31; i >= 0; i--)
                printf("%u",((numero >> i) & 1));
        printf("\n");
}
void GenBarra_Mat(uint32_t Mat[],int indice){
        uint32_t val=0;
        for (int j=0;j<indice;j++){
                val=val | (1<<j);
        }
        if (TXT_DEBUG >0)
          binary(val);
        for (int i=0;i<Mat_Alt;i++){
                Mat[i]=val;
        }
}
void Blanc_Mat(uint32_t Mat[]){
        for (int i=0;i<Mat_Alt;i++){
                Mat[i]=0b00000000000000000000000000000000;
        }
}
void Pone_Car_Mat(const uint8_t Caract[], int posicion,uint32_t Mat[]){
        for (int i=0;i<Mat_Alt;i++){
                Mat[i]=Mat[i] | (uint32_t)(Caract[i] << posicion);
        }
}
void Imprime_Mat(uint32_t Mat[]){
        printf("Matriz :\n");
        for (int i=0;i<Mat_Alt;i++){
                binary(Mat[i]);
        }
}
void Grafica_Mat(uint32_t Matriz[],int largo,int color,int borra){
        int val;
        for (int i=0;i<largo;i++){
                val=(Matriz[0]>> i) & 1;
                gpio_set_level(PIN1_RED1, val);
                val=(Matriz[1]>> i) & 1;
                gpio_set_level(PIN1_RED2, val);
                val=(Matriz[2]>> i) & 1;
                gpio_set_level(PIN2_RED1, val);
                val=(Matriz[3]>> i) & 1;
                gpio_set_level(PIN2_RED2, val);
                val=(Matriz[4]>> i) & 1;
                gpio_set_level(PIN3_RED1, val);
                val=(Matriz[5]>> i) & 1;
                gpio_set_level(PIN3_RED2, val);
                val=(Matriz[6]>> i) & 1;
                gpio_set_level(PIN4_RED1, val);
                val=(Matriz[7]>> i) & 1;
                gpio_set_level(PIN4_RED2, val);
                if(SDELAY>0)
                   vTaskDelay(SDELAY / portTICK_PERIOD_MS);
                switch (color){
                case 0:
                        gpio_set_level(PIN_RED_CLK, 1);
                        if(TXT_DEBUG>0)
                           printf("Clock RED.\n");
                        if(SDELAY>0)
                          vTaskDelay(SDELAY / portTICK_PERIOD_MS);
                        gpio_set_level(PIN_RED_CLK, 0);
                        if(SDELAY>0)
                          vTaskDelay(SDELAY / portTICK_PERIOD_MS);
                        if(borra==1){
                                gpio_set_level(PIN1_RED1, 0);
                                gpio_set_level(PIN1_RED2, 0);
                                gpio_set_level(PIN2_RED1, 0);
                                gpio_set_level(PIN2_RED2, 0);
                                gpio_set_level(PIN3_RED1, 0);
                                gpio_set_level(PIN3_RED2, 0);
                                gpio_set_level(PIN4_RED1, 0);
                                gpio_set_level(PIN4_RED2, 0);
                                if(SDELAY>0)
                                  vTaskDelay(SDELAY / portTICK_PERIOD_MS);
                                gpio_set_level(PIN_GRN_CLK, 1);
                                if(TXT_DEBUG>0)
                                   printf("Clock GREEN BLANQUEO.\n");
                                if(SDELAY>0)
                                  vTaskDelay(SDELAY / portTICK_PERIOD_MS);
                                gpio_set_level(PIN_GRN_CLK, 0);
                                if(SDELAY>0)
                                  vTaskDelay(SDELAY / portTICK_PERIOD_MS);
                        }
                        break;
                case 1:
                        gpio_set_level(PIN_GRN_CLK, 1);
                        if(TXT_DEBUG>0)
                           printf("Clock GREEN.\n");
                        if(SDELAY>0)
                          vTaskDelay(SDELAY / portTICK_PERIOD_MS);
                        gpio_set_level(PIN_GRN_CLK, 0);
                        if(SDELAY>0)
                          vTaskDelay(SDELAY / portTICK_PERIOD_MS);
                        if(borra==1){
                                gpio_set_level(PIN1_RED1, 0);
                                gpio_set_level(PIN1_RED2, 0);
                                gpio_set_level(PIN2_RED1, 0);
                                gpio_set_level(PIN2_RED2, 0);
                                gpio_set_level(PIN3_RED1, 0);
                                gpio_set_level(PIN3_RED2, 0);
                                gpio_set_level(PIN4_RED1, 0);
                                gpio_set_level(PIN4_RED2, 0);
                                if(SDELAY>0)
                                  vTaskDelay(SDELAY / portTICK_PERIOD_MS);
                                gpio_set_level(PIN_RED_CLK, 1);
                                if(TXT_DEBUG>0)
                                   printf("Clock RED BLANQUEO.\n");
                                if(SDELAY>0)
                                  vTaskDelay(SDELAY / portTICK_PERIOD_MS);
                                gpio_set_level(PIN_RED_CLK, 0);
                                if(SDELAY>0)
                                  vTaskDelay(SDELAY / portTICK_PERIOD_MS);
                        }
                        break;
                case 2:
                        gpio_set_level(PIN_RED_CLK, 1);
                        gpio_set_level(PIN_GRN_CLK, 1);
                        if(TXT_DEBUG>0)
                           printf("Clock RED-GREEN.\n");
                        if(SDELAY>0)
                          vTaskDelay(SDELAY / portTICK_PERIOD_MS);
                        gpio_set_level(PIN_RED_CLK, 0);
                        gpio_set_level(PIN_GRN_CLK, 0);
                        break;
                default:
                        gpio_set_level(PIN_RED_CLK, 1);
                        if(TXT_DEBUG>0)
                            printf("Clock RED.\n");
                        if(SDELAY>0)
                          vTaskDelay(SDELAY / portTICK_PERIOD_MS);
                        gpio_set_level(PIN_RED_CLK, 0);
                }


        }
        switch (color){
                case 0:
                        if(SDELAY>0)
                          vTaskDelay(SDELAY / portTICK_PERIOD_MS);
                        gpio_set_level(PIN_RED_STR, 1);
                        if(TXT_DEBUG>0)
                          printf("STROB RED.\n");
                        if(SDELAY>0)
                          vTaskDelay(SDELAY / portTICK_PERIOD_MS);
                        gpio_set_level(PIN_RED_STR, 0);
                        if(borra==1){
                                if(SDELAY>0)
                                  vTaskDelay(SDELAY / portTICK_PERIOD_MS);
                                gpio_set_level(PIN_GRN_STR, 1);
                                if(TXT_DEBUG>0)
                                  printf("STROB GREEN BORRA.\n");
                                if(SDELAY>0)
                                  vTaskDelay(SDELAY / portTICK_PERIOD_MS);
                                gpio_set_level(PIN_GRN_STR, 0);
                        }
                        break;
                case 1:
                        if(SDELAY>0)
                          vTaskDelay(SDELAY / portTICK_PERIOD_MS);
                        gpio_set_level(PIN_GRN_STR, 1);
                        if(TXT_DEBUG>0)
                           printf("STROB GREEN.\n");
                        if(SDELAY>0)
                          vTaskDelay(SDELAY / portTICK_PERIOD_MS);
                        gpio_set_level(PIN_GRN_STR, 0);
                        if(borra==1){
                                if(SDELAY>0)
                                  vTaskDelay(SDELAY / portTICK_PERIOD_MS);
                                gpio_set_level(PIN_RED_STR, 1);
                                if(TXT_DEBUG>0)
                                   printf("STROB RED BORRA.\n");
                                if(SDELAY>0)
                                  vTaskDelay(SDELAY / portTICK_PERIOD_MS);
                                gpio_set_level(PIN_RED_STR, 0);
                        }
                        break;
                case 2:
                        if(SDELAY>0)
                          vTaskDelay(SDELAY / portTICK_PERIOD_MS);
                        gpio_set_level(PIN_RED_STR, 1);
                        gpio_set_level(PIN_GRN_STR, 1);
                        if(TXT_DEBUG>0)
                           printf("STROB RED-GREEN.\n");
                        if(SDELAY>0)
                          vTaskDelay(SDELAY / portTICK_PERIOD_MS);
                        gpio_set_level(PIN_RED_STR, 0);
                        gpio_set_level(PIN_GRN_STR, 0);
                        break;
                default:
                        if(SDELAY>0)
                          vTaskDelay(SDELAY / portTICK_PERIOD_MS);
                        gpio_set_level(PIN_RED_STR, 1);
                        if(TXT_DEBUG>0)
                           printf("STROB RED.\n");
                        if(SDELAY>0)
                          vTaskDelay(SDELAY / portTICK_PERIOD_MS);
                        gpio_set_level(PIN_RED_STR, 0);
        }
        if(TXT_DEBUG>0)
          printf("BLK .\n");
        gpio_set_level(PIN_BLK, 0);
}

void Grafica_Matriz_Desplaza(uint32_t Matriz[],int largo,int color,int borra,int direccion){

uint32_t MatrizTemp[]={
0b00000000000000000000000000000000,
0b00000000000000000000000000000000,
0b00000000000000000000000000000000,
0b00000000000000000000000000000000,
0b00000000000000000000000000000000,
0b00000000000000000000000000000000,
0b00000000000000000000000000000000,
0b00000000000000000000000000000000
};
        for(int i=0;i<=largo;i++){
                if(direccion==0){
                MatrizTemp[0]=(uint32_t)((Matriz[0]<< (largo - i)) & 0b11111111111111111111111111111111);
                MatrizTemp[1]=(uint32_t)((Matriz[1]<< (largo - i)) & 0b11111111111111111111111111111111);
                MatrizTemp[2]=(uint32_t)((Matriz[2]<< (largo - i)) & 0b11111111111111111111111111111111);
                MatrizTemp[3]=(uint32_t)((Matriz[3]<< (largo - i)) & 0b11111111111111111111111111111111);
                MatrizTemp[4]=(uint32_t)((Matriz[4]<< (largo - i)) & 0b11111111111111111111111111111111);
                MatrizTemp[5]=(uint32_t)((Matriz[5]<< (largo - i)) & 0b11111111111111111111111111111111);
                MatrizTemp[6]=(uint32_t)((Matriz[6]<< (largo - i)) & 0b11111111111111111111111111111111);
                MatrizTemp[7]=(uint32_t)((Matriz[7]<< (largo - i)) & 0b11111111111111111111111111111111);
                }else{
                MatrizTemp[0]=(uint32_t)((Matriz[0]>> (largo - i)) & 0b11111111111111111111111111111111);
                MatrizTemp[1]=(uint32_t)((Matriz[1]>> (largo - i)) & 0b11111111111111111111111111111111);
                MatrizTemp[2]=(uint32_t)((Matriz[2]>> (largo - i)) & 0b11111111111111111111111111111111);
                MatrizTemp[3]=(uint32_t)((Matriz[3]>> (largo - i)) & 0b11111111111111111111111111111111);
                MatrizTemp[4]=(uint32_t)((Matriz[4]>> (largo - i)) & 0b11111111111111111111111111111111);
                MatrizTemp[5]=(uint32_t)((Matriz[5]>> (largo - i)) & 0b11111111111111111111111111111111);
                MatrizTemp[6]=(uint32_t)((Matriz[6]>> (largo - i)) & 0b11111111111111111111111111111111);
                MatrizTemp[7]=(uint32_t)((Matriz[7]<< (largo - i)) & 0b11111111111111111111111111111111);
                }
                if(TXT_DEBUG>0)
                  Imprime_Mat(MatrizTemp);
                Grafica_Mat(MatrizTemp,largo,color,borra);
        }
}

void Grafica_Matriz_DesplazayFunde(uint32_t Matriz[],uint32_t Matrizf[],int largo,int color,int borra){

uint32_t MatrizTemp[]={
0b00000000000000000000000000000000,
0b00000000000000000000000000000000,
0b00000000000000000000000000000000,
0b00000000000000000000000000000000,
0b00000000000000000000000000000000,
0b00000000000000000000000000000000,
0b00000000000000000000000000000000,
0b00000000000000000000000000000000
};
        for(int i=1;i<=largo;i++){
                MatrizTemp[0]=(uint32_t)((Matriz[0]<< (largo - i)) & 0b11111111111111111111111111111111);
                MatrizTemp[1]=(uint32_t)((Matriz[1]<< (largo - i)) & 0b11111111111111111111111111111111);
                MatrizTemp[2]=(uint32_t)((Matriz[2]<< (largo - i)) & 0b11111111111111111111111111111111);
                MatrizTemp[3]=(uint32_t)((Matriz[3]<< (largo - i)) & 0b11111111111111111111111111111111);
                MatrizTemp[4]=(uint32_t)((Matriz[4]<< (largo - i)) & 0b11111111111111111111111111111111);
                MatrizTemp[5]=(uint32_t)((Matriz[5]<< (largo - i)) & 0b11111111111111111111111111111111);
                MatrizTemp[6]=(uint32_t)((Matriz[6]<< (largo - i)) & 0b11111111111111111111111111111111);
                MatrizTemp[7]=(uint32_t)((Matriz[7]<< (largo - i)) & 0b11111111111111111111111111111111);
                if(TXT_DEBUG>0)
                  Imprime_Mat(MatrizTemp);
                Grafica_Mat(MatrizTemp,largo,color,borra);
        }
        if(DELAY_BANNER>0)
          vTaskDelay(DELAY_BANNER / portTICK_RATE_MS);
        for(int i=1;i<largo;i++){
                MatrizTemp[0]=(uint32_t)(((Matriz[0]<< i) | (Matrizf[0]>> (largo - i))) & 0b11111111111111111111111111111111);
                MatrizTemp[1]=(uint32_t)(((Matriz[1]<< i) | (Matrizf[1]>> (largo - i))) & 0b11111111111111111111111111111111);
                MatrizTemp[2]=(uint32_t)(((Matriz[2]<< i) | (Matrizf[2]>> (largo - i))) & 0b11111111111111111111111111111111);
                MatrizTemp[3]=(uint32_t)(((Matriz[3]<< i) | (Matrizf[3]>> (largo - i))) & 0b11111111111111111111111111111111);
                MatrizTemp[4]=(uint32_t)(((Matriz[4]<< i) | (Matrizf[4]>> (largo - i))) & 0b11111111111111111111111111111111);
                MatrizTemp[5]=(uint32_t)(((Matriz[5]<< i) | (Matrizf[5]>> (largo - i))) & 0b11111111111111111111111111111111);
                MatrizTemp[6]=(uint32_t)(((Matriz[6]<< i) | (Matrizf[6]>> (largo - i))) & 0b11111111111111111111111111111111);
                MatrizTemp[7]=(uint32_t)(((Matriz[7]<< i) | (Matrizf[7]>> (largo - i))) & 0b11111111111111111111111111111111);
                if(TXT_DEBUG>0)
                  Imprime_Mat(MatrizTemp);
                Grafica_Mat(MatrizTemp,largo,color,borra);
        }
        if(DELAY_BANNER>0)
          vTaskDelay(DELAY_BANNER / portTICK_RATE_MS);
}
void Grafica_Banner(void){
        //Blanc_Mat(Matriz);
        //Grafica_Mat(Matriz,32,2,0);
        Grafica_Matriz_DesplazayFunde(MatrizROSARIO,MatrizSMART,32,2,0);
        Blanc_Mat(Matriz);
        Grafica_Mat(Matriz,32,2,0);
}

// Callback (registered below) fired when a pulse is detected
void onBeatDetected()
{
  Serial.println("Beat!");
  Serial.print("Beat core :");
  Serial.println(xPortGetCoreID());
   BaseType_t xHigherPriorityTaskWoken;
   xEventGroupSetBitsFromISR(eglat,TASK_1_BIT, &xHigherPriorityTaskWoken);
 }

void TaskLatidos( void * pvParameters ){

    Serial.println("Task Run...");
    String taskMessage = "Task running on core ";
    taskMessage = taskMessage + xPortGetCoreID();

    while(true){
        EventBits_t x = xEventGroupWaitBits(eglat,TASK_1_BIT, pdFALSE, pdTRUE, portMAX_DELAY);
        Serial.println(taskMessage);
        Serial.println("task Beat!");
        x = xEventGroupClearBits(eglat,TASK_1_BIT);
        Serial.println(xbit2);
        Blanc_Mat(Matriz);
        Pone_Car_Mat(CaracteresArray3[0],8,Matriz);
        Grafica_Mat(Matriz,32,2,1);
        delay(400);
        Blanc_Mat(Matriz);
        Grafica_Mat(Matriz,32,2,1);
        delay(200);
     }

}

void setup()
{
    Serial.begin(115200);
   pinMode(PIN1_RED1, OUTPUT);
   pinMode(PIN1_RED2, OUTPUT);
   pinMode(PIN2_RED1, OUTPUT);
   pinMode(PIN2_RED2, OUTPUT);
   pinMode(PIN3_RED1, OUTPUT);
   pinMode(PIN3_RED2, OUTPUT);
   pinMode(PIN4_RED1, OUTPUT);
   pinMode(PIN4_RED2, OUTPUT);
   pinMode(PIN_BLK, OUTPUT);
   pinMode(PIN_RED_STR, OUTPUT);
   pinMode(PIN_RED_CLK, OUTPUT);
   pinMode(PIN_GRN_STR, OUTPUT);
   pinMode(PIN_GRN_CLK, OUTPUT);
   
   digitalWrite(PIN_BLK, LOW);
   digitalWrite(PIN_RED_CLK, LOW);
   digitalWrite(PIN_GRN_CLK, LOW);
   delay(200);
   int val=0;
   Blanc_Mat(Matriz);
   Grafica_Mat(Matriz,32,2,0);
   Grafica_Mat(MatrizCIOR,32,2,0);
   if(DELAY_BANNER>0)
      vTaskDelay(DELAY_BANNER / portTICK_RATE_MS);
   Grafica_Banner();
   Blanc_Mat(Matriz);
              
  delay(1000);
   Serial.print("Initializing MAX30100..");

    if (!ox.begin()) {
        Serial.print("FAILED: ");

        uint8_t partId = sensor.getPartId();
        if (partId == 0xff) {
            Serial.println("I2C error");
        } else {
            Serial.print("wrong part ID 0x");
            Serial.print(partId, HEX);
            Serial.print(" (expected: 0x");
            Serial.println(EXPECTED_PART_ID, HEX);
        }
        // Stop here
        for(;;);
    } else {
        Serial.println("Success");
    }

    Serial.print("Enabling HR/SPO2 mode..");
    sensor.setMode(MAX30100_MODE_SPO2_HR);
    Serial.println("done.");

    //Serial.print("Configuring LEDs biases to 50mA..");
    //sensor.setLedsCurrent(MAX30100_LED_CURR_50MA, MAX30100_LED_CURR_50MA);
    //Serial.println("done.");
    //delay(300);
    //Serial.print("Lowering the current to 7.6mA..");
    //sensor.setLedsCurrent(MAX30100_LED_CURR_7_6MA, MAX30100_LED_CURR_7_6MA);
    //Serial.println("done.");

    delay(1000);

    Serial.print("Shutting down..");
    sensor.shutdown();
    Serial.println("done.");

    eg = xEventGroupCreate();
    eglat = xEventGroupCreate();
    xbit0 = xEventGroupClearBits(eglat,TASK_1_BIT);
    xbit0 = xEventGroupClearBits(eg,TASK_1_BIT);
    Serial.print("xEventGroupClearBits even xbit0: ");
    Serial.println(xbit0);
        // The default current for the IR LED is 50mA and it could be changed
    //   by uncommenting the following line. Check MAX30100_Registers.h for all the
    //   available options.
    //ox.setIRLedCurrent(MAX30100_LED_CURR_7_6MA);
    ox.setIRLedCurrent(MAX30100_LED_CURR_24MA);
    xTaskCreatePinnedToCore(
                    TaskLatidos,   /* Function to implement the task */
                    "TaskLat", /* Name of the task */
                    1000,      /* Stack size in words */
                    NULL,       /* Task input parameter */
                    0,          /* Priority of the task */
                    NULL,       /* Task handle. */
                    LattaskCore);  /* Core where the task should run */

  Serial.println("Task created...");

    // Register a callback for the beat detection
    ox.setOnBeatDetectedCallback(onBeatDetected);
    delay(1000);
    thresholdt=touchRead(T0);
    Serial.println(thresholdt);  // get value using T0 
    delay(1000);
    thresholdt=touchRead(T0);
    Serial.println(thresholdt);
    touchAttachInterrupt(T0, gotTouch,thresholdt - 1 );
    printf("\n ESP32 Touch Interrupt \n");
    printf("Fin SETUP.\n");
    Serial.print("setup core :");
    Serial.println(xPortGetCoreID());
    delay(500);
}

          
void loop()
{
    // Make sure to call update as fast as possible
    /* wait forever until event bit of task 1 is set */
    xbit1 = xEventGroupWaitBits(eg, TASK_1_BIT, pdFALSE, pdTRUE, portMAX_DELAY);
    if(xbit1 == 1){
       //Serial.print("Loop has even xbit1: ");
       //Serial.println(xbit1);
       ox.update();

       // Asynchronously dump heart rate and oxidation levels to the serial
       // For both, a value of 0 means "invalid"
       if(millis() - tsLastReport > REPORTING_PERIOD_MS){
          Serial.print("Heart rate:");
          Serial.print(ox.getHeartRate());
          Serial.print("bpm / SpO2:");
          Serial.print(ox.getSpO2());
          Serial.println("%");
          tsLastReport = millis();
       }
       tread0=touchRead(T0);
       if(tread0 > thresholdt){
          Serial.print("thresholdt..");
          Serial.println(tread0);  // get value using T0    
          xbit2 = xEventGroupClearBits(eg,TASK_1_BIT);
          Serial.println(xbit2);
          Serial.print("Shutting down..");
          sensor.shutdown();
          Serial.println("done.");
          touchAttachInterrupt(T0, gotTouch,thresholdt - 1 );
          delay(1000);
       }
    }else{
       Serial.print("Loop has even <> 1 xbit1: ");
       Serial.println(xbit1);
       delay(1000);
    }
    //Serial.print("loop core :");
    //Serial.println(xPortGetCoreID());
}

void gotTouch(){
  Serial.println("ISR Touched\n");
  Serial.print("gottouch core :");
  Serial.println(xPortGetCoreID());
  tread0=touchRead(T0);
  Serial.println(tread0);  // get value using T0 
  tch++;
  if (tch == 1){
   touchAttachInterrupt(T0, gotTouch, 0);
   Serial.println("ISR Touched\n");
   BaseType_t xHigherPriorityTaskWoken;
   Serial.print("ISR Resuming normal operation..");
   sensor.resume();
   delay(500);
   Serial.println("ISR done.");
   //ox.setIRLedCurrent(MAX30100_LED_CURR_7_6MA);
   ox.setIRLedCurrent(MAX30100_LED_CURR_24MA);
   // Register a callback for the beat detection
   ox.setOnBeatDetectedCallback(onBeatDetected);
   sensor.resetFifo();
   Serial.print("ISR ESP32 Event TASK_1_BIT\n");
   xEventGroupSetBitsFromISR(eg,TASK_1_BIT, &xHigherPriorityTaskWoken);
   Serial.print("ISR ESP32 Touch Interrupt\n");
   tch=0;
  }else{
    Serial.print("ISR ESP32 Touch Interrupt duplicado");
  }
  
}

