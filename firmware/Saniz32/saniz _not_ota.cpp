/* GPIO Example

   This example code is in the Public Domain (or CC0 licensed, at your option.)

   Unless required by applicable law or agreed to in writing, this
   software is distributed on an "AS IS" BASIS, WITHOUT WARRANTIES OR
   CONDITIONS OF ANY KIND, either express or implied.
*/
#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include <esp_system.h>
#include "freertos/queue.h"
#include "driver/gpio.h"
#include <button.h>
#include <esp_log.h>
#include <esp_idf_lib_helpers.h>
#include <ultrasonic.h>


#include <esp_spi_flash.h>

#include <FastLED.h>
#include <FX.h>

//#include "u8g2_esp32_hal.h"


extern "C"
{
    #include "MLX90614_API.h"
    #include "MLX90614_SMBus_Driver.h"
    #include "u8g2.h"
    #include <u8g2_esp32_hal.h>
}


#define RELAY1      GPIO_NUM_32
#define RELAY2      GPIO_NUM_33
#define ONBOARD_LED  GPIO_NUM_2
#define BUZZER      GPIO_NUM_23

#define IR_SENSE    GPIO_NUM_5
#define PUSH_BTN    GPIO_NUM_27

#define GPIO_OUTPUT_PIN_SEL  ((1ULL<<RELAY1) | (1ULL<<RELAY2)| (1ULL<<ONBOARD_LED)|(1ULL<<BUZZER))
#define REED_SW     GPIO_NUM_4
#define GPIO_INPUT_PIN_SEL  ((1ULL<<REED_SW) | (1ULL<<IR_SENSE))
#define ESP_INTR_FLAG_DEFAULT 0

//I2C macros
#define I2C_SPEED 100000 //100Khz
#define I2C_PORT I2C_NUM_0
#define SDA_GPIO GPIO_NUM_21 //13 // sda for MLX90614
#define SCL_GPIO GPIO_NUM_22 //12 // scl for MLX90614

//MLX90614 Macros
#define MLX90614_DEFAULT_ADDRESS 0x5A // default chip address(slave address) of MLX90614
#define MLX90614_VCC_GPIO GPIO_NUM_15 // vcc for MLX90614
#define TEMP_MEASURE_TIME_SEC       3       


//Relay Macros
#define RELAY1_ON       gpio_set_level(RELAY1, 1)
#define RELAY1_OFF      gpio_set_level(RELAY1,0)

#define RELAY2_ON       gpio_set_level(RELAY2, 1)
#define RELAY2_OFF      gpio_set_level(RELAY2,0)

#define LED_ON          gpio_set_level(ONBOARD_LED,1);
#define LED_OFF         gpio_set_level(ONBOARD_LED,0);

//Ultrasonic HC-SR-04 macros
#define MAX_DISTANCE_CM 400 // 4m max
#define TRIGGER_GPIO    GPIO_NUM_19
#define ECHO_GPIO       GPIO_NUM_18

//Buzzer Macros
#define BUZZER_ON       gpio_set_level(BUZZER,1);
#define BUZZER_OFF      gpio_set_level(BUZZER,0);
#define BUZZER_DUR_MS   500

//LED macros
#define NUM_LEDS        3
#define DATA_PIN        13
#define LED_TYPE        WS2812
#define COLOR_ORDER     GRB

#define RGB_EVENT_SANITIZING    1
#define RGB_EVENT_PERSON_DETECT 2
#define RGB_EVENT_MEAS_TEMP     3
#define RGB_EVENT_SANITIZING_CONT   4
#define RGB_EVENT_STICK_DETACH      5
#define RGB_EVENT_STICK_ATTACH      6
#define RGB_EVENT_IDLE          0

#define RGB_ANIM_CONTINUOS_SEC  255

//OLED macros
#define OLED_EVT_SHOW_To    1
#define OLED_EVT_SHOW_Ta    2
#define OLED_EVT_SANIZ      3
#define OLED_EVT_DETECT     4
#define OLED_EVT_IDLE       5     

CRGB rgb_led[NUM_LEDS];

#define N_COLORS 17
static const CRGB colors[N_COLORS] = { 
  CRGB::Red,
  CRGB::Green,
  CRGB::Blue,
  CRGB::White,
  CRGB::AliceBlue,
  CRGB::ForestGreen,
  CRGB::Lavender,
  CRGB::MistyRose,
  CRGB::DarkOrchid,
  CRGB::DarkOrange,
  CRGB::Black,
  CRGB::Teal,
  CRGB::Violet,
  CRGB::Lime,
  CRGB::Chartreuse,
  CRGB::BlueViolet,
  CRGB::Aqua
};

typedef struct {
  const char *name;
  uint8_t event_id;
  int   mode;
  int   secs; 
  uint32_t color;
  int speed;
  uint8_t max_brightness;
} animate_rgb_mode_t;

typedef struct{
  uint8_t id;
  float ta;
  float to;
} mlx_temp_data_t;

typedef struct{
    uint8_t event_id;
    int secs;
}oled_show_data_t;

//FX modes for various event
static const animate_rgb_mode_t event_rgb_modes[] = {
    {"kept empty", 0,0,0,0,0},
    {"Sanitizing Animation", RGB_EVENT_SANITIZING, FX_MODE_FADE, 3, GREEN, 500, 255},
    {"Person Detected Animation", RGB_EVENT_PERSON_DETECT, FX_MODE_POLICE_ALL, 10,0, 255, 255},
    {"Measuring Temerature Animation", RGB_EVENT_MEAS_TEMP, FX_MODE_BOUNCINGBALLS, 3, YELLOW, 255,50},
    {"Sanitizing Animation Continuous", RGB_EVENT_SANITIZING_CONT, FX_MODE_FADE, 255, GREEN, 500, 255},
    {"Sanitizing Stick Detached", RGB_EVENT_STICK_DETACH, FX_MODE_BLINK, 1, CYAN, 500, 255},
    {"Sanitizing Stick Attached", RGB_EVENT_STICK_ATTACH, FX_MODE_BLINK, 1, RED, 500, 255}
    
};
static const animate_rgb_mode_t idle_rgb_mode = {"Idle Animation", 0, FX_MODE_SINEWAVE, 0, RED, 50, 50};

static const oled_show_data_t  oled_events[] = {
    {0,0},
    {OLED_EVT_SHOW_To, 3},
    {OLED_EVT_SHOW_Ta, 3},
    {OLED_EVT_SANIZ, 3},
    {OLED_EVT_DETECT, 10},
    {OLED_EVT_IDLE, 0},
};

static xQueueHandle gpio_evt_queue = NULL;
static xQueueHandle animate_rgb_queue = NULL;
static xQueueHandle temp_data_queue = NULL;

SemaphoreHandle_t mutex_i2c = NULL;

static const char *TAG = "button_example";
static const char *states[] = {
    [BUTTON_PRESSED]      = "pressed",
    [BUTTON_RELEASED]     = "released",
    [BUTTON_CLICKED]      = "clicked",
    [BUTTON_PRESSED_LONG] = "pressed long",
};

static button_t btnStart;
static TaskHandle_t xIrSensedNotifyTask_handle;
static TaskHandle_t xGetTempNotifyTask_handle;
//static TaskHandle_t xReedSensedNotifyTask_handle;
    
bool saniz_stick_attached;


static void on_button(button_t *btn, button_state_t state)
{
    ESP_LOGI(TAG, "%s button %s", btn == &btnStart? "First" : "Second", states[state]);

    if(state == BUTTON_PRESSED){
                LED_ON;
                printf("Sanitizing..\n");
                RELAY1_ON;
                RELAY2_ON;

                uint8_t event_id = RGB_EVENT_SANITIZING_CONT;
                xQueueSendToBack(animate_rgb_queue, &event_id, NULL);
    }
    else if(state == BUTTON_RELEASED){
                LED_OFF;
                printf("Done Sanitizing \n");
                RELAY1_OFF;
                RELAY2_OFF;
                
                uint8_t event_id = RGB_EVENT_IDLE;    
                xQueueSendToBack(animate_rgb_queue, &event_id, NULL);
    }
}

static void IRAM_ATTR ir_sense_isr_handler(void* arg)
{
   // uint32_t gpio_num = (uint32_t) arg;
   // xQueueSendFromISR(gpio_evt_queue, &gpio_num, NULL);

    BaseType_t xHigherPriorityTaskWoken = pdFALSE;

    configASSERT( xIrSensedNotifyTask_handle != NULL );
    vTaskNotifyGiveFromISR(xIrSensedNotifyTask_handle, &xHigherPriorityTaskWoken);
   // portYIELD_FROM_ISR( xHigherPriorityTaskWoken );
    portYIELD_FROM_ISR();
}

// static void IRAM_ATTR reed_sense_isr_handler(void* arg)
// {
//     BaseType_t xHigherPriorityTaskWoken = pdFALSE;

//     configASSERT( xReedSensedNotifyTask_handle != NULL );
//     vTaskNotifyGiveFromISR(xReedSensedNotifyTask_handle, &xHigherPriorityTaskWoken);

//     portYIELD_FROM_ISR();
// }

extern "C" {
  void app_main();
}

void blinkLeds_simple(void *pvParameters){

 	while(1){

		for (int j=0;j<N_COLORS;j++) {
			printf("blink leds\n");

			// for (int i=0;i<NUM_LEDS;i++) {
			//   rgb_led[i] = colors[j];
			// }
            rgb_led[2] = colors[j];
			FastLED.show();
			delay(1000);
		};
	}
};

static void ir_sensed_task(void* arg)
{
    //uint32_t io_num;
    gpio_num_t io_num;
    for(;;) {
        if(xQueueReceive(gpio_evt_queue, &io_num, portMAX_DELAY)) {
            printf("GPIO[%d] intr, val: %d\n", io_num, gpio_get_level(io_num));

            if(gpio_get_level(io_num) == 0)
            {
               LED_ON;
                printf("Sanitizing..\n");
                RELAY1_ON;
                RELAY2_ON;
              
            }
            else  {
                  //  LED_OFF;
                    printf("Done Sanitizing \n");
                    RELAY1_OFF;
                    RELAY2_OFF;
            }

        }
    }
}
static bool sanitizing=false;
static bool halt_temp_measuring = false;
static void sanitize_task(void *params)
{
    
    if(saniz_stick_attached == false)
    vTaskDelete(NULL);

    uint64_t time_start = esp_timer_get_time();

    while (gpio_get_level(IR_SENSE) == 0)
    {
        if(esp_timer_get_time() >= (time_start + (TEMP_MEASURE_TIME_SEC*1000000)))
        {
            LED_ON;
            printf("Sanitizing..\n");
            RELAY1_ON;
            RELAY2_ON;

             uint8_t event_id = RGB_EVENT_SANITIZING;
            xQueueSendToBack(animate_rgb_queue, &event_id, NULL);
        }

        vTaskDelay(pdMS_TO_TICKS(200));
    }

    vTaskDelete(NULL);
    
}
static void ReedSensedNotifyTask(void* args)
{
    // uint32_t notificationValue;
    

     for(;;)
     {
      //   notificationValue = ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(500) );

     //    if(notificationValue == 1)
         {
         //    gpio_intr_disable(REED_SW);
             int current_level = gpio_get_level(REED_SW);

              vTaskDelay(pdMS_TO_TICKS(200));

            if(gpio_get_level(REED_SW) == current_level)
            {
                if((current_level == 0) && (saniz_stick_attached == false) )
                {
                    saniz_stick_attached = true;

                    printf("\nAttached\n");
                    
                    uint8_t event_id = RGB_EVENT_STICK_ATTACH;
                    xQueueSendToBack(animate_rgb_queue, &event_id, NULL);
                }
                else if(( current_level == 1) && (saniz_stick_attached == true))
                {
                    
                    saniz_stick_attached = false;

                    printf("\nDettached\n");

                    uint8_t event_id = RGB_EVENT_STICK_DETACH;
                    xQueueSendToBack(animate_rgb_queue, &event_id, NULL);
                } 
            }

        //    gpio_intr_enable(REED_SW);
         }
     }
}

static void IrSensedNotifyTask(void* args)
{
    uint32_t notificationValue;

    for(;;)
    {
        notificationValue = ulTaskNotifyTake(pdTRUE, pdMS_TO_TICKS(500) );
        
        if(notificationValue == 1)
        {
            gpio_intr_disable(IR_SENSE);
            int current_level = gpio_get_level(IR_SENSE);
            
            vTaskDelay(pdMS_TO_TICKS(200));

            if(gpio_get_level(IR_SENSE) == current_level){
                if(current_level == 0)
                {
                        // LED_ON;
                        // printf("Sanitizing..\n");
                        // RELAY1_ON;
                        // RELAY2_ON;

                        sanitizing = true;    
                      //  uint8_t event_id = RGB_EVENT_SANITIZING;
                       // xQueueSendToBack(animate_rgb_queue, &event_id, NULL);
                        
                       uint8_t event_id = RGB_EVENT_MEAS_TEMP;
                       xQueueSendToBack(animate_rgb_queue, &event_id, NULL);

                        xTaskCreate(sanitize_task, "sanitize_task", 1048, NULL, 10, NULL);
                        xTaskNotifyGive(xGetTempNotifyTask_handle);
                }
                else{
                   //  LED_OFF;
                            printf("Done Sanitizing \n");
                            RELAY1_OFF;
                            RELAY2_OFF;

                            halt_temp_measuring=true;
                }
            }
            gpio_intr_enable(IR_SENSE);

        } 
    }
}
static void detect_person(void *args)
{
    
      ultrasonic_sensor_t sensor = {
        .trigger_pin = TRIGGER_GPIO,
        .echo_pin = ECHO_GPIO
    };

    ultrasonic_init(&sensor);

    uint32_t distances[] ={0,0,0};
    uint32_t distance=0;
    int dst_ptr=0;

    int detected = 0;

    while(true)
    {

        esp_err_t res = ultrasonic_measure_cm(&sensor, MAX_DISTANCE_CM, &distance);

        if(res == ESP_OK)
        {
            printf("Distance: %d cm\n", distance);
            distances[dst_ptr] = distance;
            dst_ptr++;
            //printf("pointer =  %d \n", dst_ptr);
            if(dst_ptr >=3)
            {
              //   printf("pointer =  %d \n", dst_ptr);
                uint32_t diff=0;

                for(int i=2; i>=1; i--)
                {
                    diff = distances[i-1] - distances[i];

                    if((diff > 2) && (diff <= 50)) detected++;
                    else
                    {
                        if(detected >0)
                        detected--;
                        else detected=0;
                    } 

                   //  printf("Difference: %d cm\n", diff);

                }
               memset(distances,0, sizeof distances);
               dst_ptr=0;
               distance=0;
              // printf("\n\n");

            }

            if(detected >= 2)
            {
                printf("detected !\n");
                LED_ON;
                detected=0;
                BUZZER_ON;
                 uint8_t event_id = RGB_EVENT_PERSON_DETECT;

                mlx_temp_data_t oled_data;

                oled_data.id = OLED_EVT_DETECT;
                xQueueSendToBack(animate_rgb_queue, &event_id, NULL);
                
                xQueueSendToBack(temp_data_queue, &oled_data, NULL);
               // xTaskNotifyGive(xGetTempNotifyTask_handle);

                vTaskDelay(pdMS_TO_TICKS(BUZZER_DUR_MS));
                BUZZER_OFF;
              //  LED_OFF;
            }
            else
            {
              //  LED_OFF;
            } 
        }
        else{
             printf("Error %d: ", res);
                switch (res)
                {
                    case ESP_ERR_ULTRASONIC_PING:
                        printf("Cannot ping (device is in invalid state)\n");
                        break;
                    case ESP_ERR_ULTRASONIC_PING_TIMEOUT:
                        printf("Ping timeout (no device found)\n");
                        break;
                    case ESP_ERR_ULTRASONIC_ECHO_TIMEOUT:
                        printf("Echo timeout (i.e. distance too big)\n");
                        break;
                    default:
                        printf("%s\n", esp_err_to_name(res));
                }
        }

          vTaskDelay(pdMS_TO_TICKS(200));
    }

}
static void animate_rgb_leds (void *args)
{
  
  WS2812FX ws2812fx;
  WS2812FX::Segment *segments = ws2812fx.getSegments();

  uint64_t event_anim_time=0;
  uint8_t event_anim_on = 0;

  uint8_t   event_id;

  ws2812fx.init(NUM_LEDS, rgb_led, false); // type was configured before
  ws2812fx.setBrightness(idle_rgb_mode.max_brightness);
//   ws2812fx.setMode(0,IDLE_FX);
//   segments[0].colors[0] = RED;
//   segments[0].speed = 100;
  //segments[0].options = FADE_FAST;

  ws2812fx.setMode(0,idle_rgb_mode.mode);

   if(saniz_stick_attached == true)
            segments[0].colors[0] = idle_rgb_mode.color;
  else segments[0].colors[0] = CYAN;
  
  segments[0].speed = idle_rgb_mode.speed;
  

  while(1)
  {
    
    if(xQueueReceive(animate_rgb_queue, &event_id, NULL))
      {
          printf("indicate event on rgb strip, Event: %d\n",(int) event_id );

          /*if(event_id == RGB_EVENT_SANITIZING)
          {

              ws2812fx.setMode(0,event_rgb_modes[RGB_EVENT_SANITIZING].mode);
              segments[0].colors[0] = event_rgb_modes[RGB_EVENT_SANITIZING].color;
              segments[0].speed = event_rgb_modes[RGB_EVENT_SANITIZING].speed;

              ws2812fx.setBrightness(event_rgb_modes[RGB_EVENT_SANITIZING].max_brightness);  

                
                event_anim_time  = esp_timer_get_time() + event_rgb_modes[RGB_EVENT_SANITIZING].secs * 1000000L;
                event_anim_on=1;
          }
          else if(event_id == RGB_EVENT_PERSON_DETECT)
          {

              ws2812fx.setMode(0,event_rgb_modes[RGB_EVENT_PERSON_DETECT].mode);
              segments[0].colors[0] = event_rgb_modes[RGB_EVENT_PERSON_DETECT].color;
              segments[0].speed = event_rgb_modes[RGB_EVENT_PERSON_DETECT].speed;

              ws2812fx.setBrightness(event_rgb_modes[RGB_EVENT_PERSON_DETECT].max_brightness);  
                
                event_anim_time  = esp_timer_get_time() + event_rgb_modes[RGB_EVENT_PERSON_DETECT].secs * 1000000L;
                event_anim_on=1;
          }
          else if(event_id == RGB_EVENT_MEAS_TEMP)
          {
              ws2812fx.setMode(0,event_rgb_modes[RGB_EVENT_MEAS_TEMP].mode);
              segments[0].colors[0] = event_rgb_modes[RGB_EVENT_MEAS_TEMP].color;
              segments[0].speed = event_rgb_modes[RGB_EVENT_MEAS_TEMP].speed;

              ws2812fx.setBrightness(event_rgb_modes[RGB_EVENT_MEAS_TEMP].max_brightness);

                event_anim_time  = esp_timer_get_time() + event_rgb_modes[RGB_EVENT_MEAS_TEMP].secs * 1000000L;
                event_anim_on=1;
          }
          else if(event_id == RGB_EVENT_SANITIZING_CONT)
          {

              ws2812fx.setMode(0,event_rgb_modes[RGB_EVENT_SANITIZING_CONT].mode);
              segments[0].colors[0] = event_rgb_modes[RGB_EVENT_SANITIZING_CONT].color;
              segments[0].speed = event_rgb_modes[RGB_EVENT_SANITIZING_CONT].speed;

              ws2812fx.setBrightness(event_rgb_modes[RGB_EVENT_SANITIZING_CONT].max_brightness);  

                
                event_anim_time  = esp_timer_get_time() + event_rgb_modes[RGB_EVENT_SANITIZING_CONT].secs * 1000000L;
                event_anim_on=1;
          }

            if(event_rgb_modes[event_id].secs == RGB_ANIM_CONTINUOS_SEC)
            event_anim_on=0;
            else event_anim_on=1;
            */

           if((event_id == RGB_EVENT_SANITIZING)||
           (event_id == RGB_EVENT_SANITIZING_CONT)||
           (event_id == RGB_EVENT_PERSON_DETECT)||
           (event_id == RGB_EVENT_MEAS_TEMP)||
           (event_id == RGB_EVENT_IDLE)||
           (event_id == RGB_EVENT_STICK_DETACH)||
           (event_id == RGB_EVENT_STICK_ATTACH))
           {
                ws2812fx.setMode(0,event_rgb_modes[event_id].mode);
                segments[0].colors[0] = event_rgb_modes[event_id].color;
                segments[0].speed = event_rgb_modes[event_id].speed;

                ws2812fx.setBrightness(event_rgb_modes[event_id].max_brightness);  

                if(event_rgb_modes[event_id].secs != RGB_ANIM_CONTINUOS_SEC)
                {
                    event_anim_time  = esp_timer_get_time() + event_rgb_modes[event_id].secs * 1000000L;
                    event_anim_on=1;
                }
                else{
                    event_anim_on=0;
                }

                if(saniz_stick_attached == false)
                {
                    if(event_id == RGB_EVENT_IDLE)
                    segments[0].colors[0] = CYAN;
                    else if(event_id == RGB_EVENT_MEAS_TEMP)
                    segments[0].colors[0] = CYAN;
                }
           }


      }

        if((event_anim_time <= esp_timer_get_time()) && (event_anim_on==1))
        {
            ws2812fx.setMode(0,idle_rgb_mode.mode);

            if(saniz_stick_attached == true)
            segments[0].colors[0] = idle_rgb_mode.color;
            else segments[0].colors[0] = CYAN;

            segments[0].speed = idle_rgb_mode.speed;
            event_anim_on=0;

            ws2812fx.setBrightness(idle_rgb_mode.max_brightness);

            printf("entering event animation end");
        }


        ws2812fx.service();
      vTaskDelay(10 / portTICK_PERIOD_MS); /*10ms*/
  }

}
void display_on_oled(void *pvParams)
{
    u8g2_esp32_hal_t u8g2_esp32_hal = U8G2_ESP32_HAL_DEFAULT;
	u8g2_esp32_hal.sda   = SDA_GPIO;
	u8g2_esp32_hal.scl  = SCL_GPIO;
    u8g2_esp32_hal.clk = I2C_SPEED;

    u8g2_t u8g2;   
    mlx_temp_data_t que_temp_data;
    oled_show_data_t  oled_show_data;

    char Strbuff[20];
    
    if(mutex_i2c != NULL)
    {
        if(xSemaphoreTake(mutex_i2c, portMAX_DELAY))
        {
            u8g2_esp32_hal_init(u8g2_esp32_hal);

            // a structure which will contain all the data for one display
            u8g2_Setup_ssd1306_i2c_128x32_univision_f(
                &u8g2,
                U8G2_R0,
                //u8x8_byte_sw_i2c,
                u8g2_esp32_i2c_byte_cb,
                u8g2_esp32_gpio_and_delay_cb);  // init u8g2 structure
            u8x8_SetI2CAddress(&u8g2.u8x8,0x78);

            ESP_LOGI(TAG, "u8g2_InitDisplay");
            u8g2_InitDisplay(&u8g2); // send init sequence to the display, display is in sleep mode after this,

            printf("ikde aalo");

            ESP_LOGI(TAG, "u8g2_SetPowerSave");
            u8g2_SetPowerSave(&u8g2, 0); // wake up display
            ESP_LOGI(TAG, "u8g2_ClearBuffer");
            u8g2_ClearBuffer(&u8g2);
            // ESP_LOGI(TAG, "u8g2_DrawBox");
            // u8g2_DrawBox(&u8g2, 0, 26, 80,6);
            // u8g2_DrawFrame(&u8g2, 0,26,100,6);

            ESP_LOGI(TAG, "u8g2_SetFont");
            u8g2_SetFont(&u8g2, u8g2_font_ncenB14_tr);
            ESP_LOGI(TAG, "u8g2_DrawStr");
            u8g2_DrawStr(&u8g2, 2,17,"Saniz Mark II");
            ESP_LOGI(TAG, "u8g2_SendBuffer");
            u8g2_SendBuffer(&u8g2);
            
            vTaskDelay(pdMS_TO_TICKS(1000));

            u8g2_ClearBuffer(&u8g2);
            u8g2_DrawStr(&u8g2, 2,17,"Kaustubh T");
            u8g2_SendBuffer(&u8g2);    

            vTaskDelay(pdMS_TO_TICKS(1000));
            u8g2_SetPowerSave(&u8g2, 1);  //sleep

            xSemaphoreGive(mutex_i2c);
            printf("\nDone Display Init\n");
        }
    }

    uint64_t max_show_time=0;
    bool display_on = false;

    while(1)
    {
        if(mutex_i2c != NULL)
        {
            if(xQueueReceive(temp_data_queue, &que_temp_data, pdMS_TO_TICKS(200)))
            {              
                oled_show_data = oled_events[que_temp_data.id];
                
                if(xSemaphoreTake(mutex_i2c, (TickType_t)pdMS_TO_TICKS(500)))
                {
                    max_show_time = esp_timer_get_time() + (1000000 * oled_show_data.secs);
                    display_on=true;

                    u8g2_SetPowerSave(&u8g2, 0); // wake up display
                    u8g2_ClearBuffer(&u8g2);

                    if(que_temp_data.id == OLED_EVT_SHOW_To)
                    {
                        u8g2_SetFont(&u8g2, u8g2_font_helvB24_tf);
                        sprintf(Strbuff, "%.1f\xb0 F", que_temp_data.to);
                        u8g2_DrawStr(&u8g2, 2,30, Strbuff);
                        u8g2_SendBuffer(&u8g2);
                    }
                    else if(que_temp_data.id == OLED_EVT_SHOW_Ta)
                    {
                        sprintf(Strbuff, "%.1f\xb0 F", que_temp_data.ta);
                        u8g2_SetFont(&u8g2, u8g2_font_helvB24_tf);
                        u8g2_DrawStr(&u8g2, 2,30, Strbuff);
                        u8g2_SendBuffer(&u8g2);
                    }
                    else if(que_temp_data.id == OLED_EVT_DETECT)
                    {
                       // int quespaces = uxQueueSpacesAvailable(temp_data_queue);
                        int i=0, j=0;

                      //  printf("\r\n que = %d \n", quespaces);

                        u8g2_SetFont(&u8g2, u8g2_font_open_iconic_arrow_4x_t);
                        sprintf(Strbuff, "\x0048  \x0048  \x0048  \x0048  \x0048  ");
                        for(j=0; j<=5; j++)
                        {
                            for(i=0; i <=30; i+=10)
                            {
                                u8g2_ClearBuffer(&u8g2); 
                                u8g2_DrawStr(&u8g2, 2,i, Strbuff);
                                u8g2_SendBuffer(&u8g2);
                                
                                //if (uxQueueSpacesAvailable(temp_data_queue) != quespaces)
                                if(sanitizing == true)
                                {
                                  //  printf("\nnew message!\n");
                                    j=6;
                                    sanitizing = false;
                                    break;
                                }

                                vTaskDelay(pdMS_TO_TICKS(50));
                            }
                            for(i=30; i >=10; i-=10)
                            {
                                u8g2_ClearBuffer(&u8g2); 
                                u8g2_DrawStr(&u8g2, 2,i, Strbuff);
                                u8g2_SendBuffer(&u8g2);
                                
                                //if (uxQueueSpacesAvailable(temp_data_queue) != quespaces)
                                if(sanitizing == true)
                                {
                                  //  printf("\nnew message!\n");
                                    j=6;
                                    sanitizing = false;
                                    break;
                                }

                                vTaskDelay(pdMS_TO_TICKS(50));
                            }                            
                            
                        }

                         u8g2_ClearBuffer(&u8g2); 
                         u8g2_DrawStr(&u8g2, 2,30, Strbuff);
                         u8g2_SendBuffer(&u8g2);
                     //   printf("\r\n que = %d \n", quespaces);
                       
                    } 

                 
                    
                    xSemaphoreGive(mutex_i2c);
                   
                }
                
              //  vTaskDelay(pdMS_TO_TICKS(50));
            }
            if(display_on == true)
            {
                if(esp_timer_get_time() >= max_show_time)
                {
                        u8g2_SetPowerSave(&u8g2, 1);  //sleep
                        display_on = false;
                        printf("\nsleeping\n");
                }

            }
                
        }
        
    }
}
void mlx90614_get_temp(void *pvPatameters)
{
       // setup gpio for MLX90614
    gpio_pad_select_gpio(MLX90614_VCC_GPIO);
    gpio_set_direction(MLX90614_VCC_GPIO, GPIO_MODE_OUTPUT);
    gpio_set_level(MLX90614_VCC_GPIO, 1);
    vTaskDelay(1000/portTICK_RATE_MS);
    
     if(xSemaphoreTake(mutex_i2c, portMAX_DELAY))
     {
         MLX90614_SMBusInit(SDA_GPIO, SCL_GPIO, I2C_SPEED, I2C_PORT); // sda scl and 100kHz
         xSemaphoreGive(mutex_i2c);
         printf("\r\nDone mlx90614 Init");
     }

    float to = 0; // temperature of object
    float ta = 0; // temperature of ambient
    //uint16_t dumpInfo = 0;
    mlx_temp_data_t temp_data;
    int64_t measure_time;
   
    while(1)
    {
        ulTaskNotifyTake(pdTRUE, portMAX_DELAY);
        halt_temp_measuring = false;
        measure_time = esp_timer_get_time() + (TEMP_MEASURE_TIME_SEC * 1000000);
       
        while (measure_time >= esp_timer_get_time())
        {
            if(mutex_i2c != NULL)
            {
                
                if(xSemaphoreTake(mutex_i2c, (TickType_t)pdMS_TO_TICKS(500)))
                {
                    MLX90614_GetTo(MLX90614_DEFAULT_ADDRESS, &to);
                    MLX90614_GetTa(MLX90614_DEFAULT_ADDRESS, &ta);

                    temp_data.to = MLX90614_TemperatureInFahrenheit(to);
                    temp_data.ta = MLX90614_TemperatureInFahrenheit(ta);
                    temp_data.id = OLED_EVT_SHOW_To;
                // MLX90614_GetTa(MLX90614_DEFAULT_ADDRESS, &dumpInfo);
                    printf("log:%lf %lf\r\n", to, ta);

                    xSemaphoreGive(mutex_i2c);
                    xQueueSendToBack(temp_data_queue, &temp_data,NULL);
                    
                    vTaskDelay(pdMS_TO_TICKS(50));
                }

            }

            if(halt_temp_measuring == true)
                { printf("\nhalting");break;}

        }
        printf("\nnot halting");
       
    }

}
void ultrasonic_test(void *pvParameters)
{
    ultrasonic_sensor_t sensor = {
        .trigger_pin = TRIGGER_GPIO,
        .echo_pin = ECHO_GPIO
    };

    ultrasonic_init(&sensor);

    while (true)
    {
        float distance;
        esp_err_t res = ultrasonic_measure(&sensor, MAX_DISTANCE_CM, &distance);
        if (res != ESP_OK)
        {
            printf("Error %d: ", res);
            switch (res)
            {
                case ESP_ERR_ULTRASONIC_PING:
                    printf("Cannot ping (device is in invalid state)\n");
                    break;
                case ESP_ERR_ULTRASONIC_PING_TIMEOUT:
                    printf("Ping timeout (no device found)\n");
                    break;
                case ESP_ERR_ULTRASONIC_ECHO_TIMEOUT:
                    printf("Echo timeout (i.e. distance too big)\n");
                    break;
                default:
                    printf("%s\n", esp_err_to_name(res));
            }
        }
        else
            printf("Distance: %0.04f m\n", distance);

        vTaskDelay(pdMS_TO_TICKS(500));
    }
}
void app_main(void)
{
    // First button connected between GPIO and GND
    // pressed logic level 0, no autorepeat
    btnStart.gpio = PUSH_BTN;
    btnStart.pressed_level = 0;
    btnStart.internal_pull = true;
    btnStart.autorepeat = false;
    btnStart.callback = on_button;
    
    gpio_config_t io_conf;
    //disable interrupt
    io_conf.intr_type = (gpio_int_type_t)GPIO_PIN_INTR_DISABLE;
    //set as output mode
    io_conf.mode = GPIO_MODE_OUTPUT;
    //bit mask of the pins that you want to set,e.g.GPIO18/19
    io_conf.pin_bit_mask = GPIO_OUTPUT_PIN_SEL;
    //disable pull-down mode
    io_conf.pull_down_en = (gpio_pulldown_t)1;
    //disable pull-up mode
    io_conf.pull_up_en = (gpio_pullup_t)0;
    //configure GPIO with the given settings
    gpio_config(&io_conf);

    // //interrupt of rising edge
     io_conf.intr_type = (gpio_int_type_t)GPIO_PIN_INTR_ANYEDGE;
    // //bit mask of the pins, use GPIO4/5 here
     io_conf.pin_bit_mask = GPIO_INPUT_PIN_SEL;
    // //set as input mode    
     io_conf.mode = GPIO_MODE_INPUT;
    // //enable pull-up mode
     io_conf.pull_up_en=(gpio_pullup_t)1;
     io_conf.pull_down_en=(gpio_pulldown_t)0;

     gpio_config(&io_conf);

    //change gpio intrrupt type for one pin
     gpio_set_intr_type(REED_SW, GPIO_INTR_DISABLE);

    //create a queue to handle gpio event from isr
  //  gpio_evt_queue = xQueueCreate(10, sizeof(uint32_t));

    //Create queue to handle different rgb patterns for various event
    animate_rgb_queue = xQueueCreate(10, sizeof(uint8_t));
    
    //start gpio task
   // xTaskCreate(ir_sensed_task, "ir_sensed_task", 2048, NULL, 10, NULL);

    saniz_stick_attached = false;
    xTaskCreate(IrSensedNotifyTask, "IrSensedNotifyTask", 2048, NULL, 12, &xIrSensedNotifyTask_handle);
    xTaskCreate(ReedSensedNotifyTask, "ReedSensedNotifyTask", 2048, NULL, 12, NULL /*&xReedSensedNotifyTask_handle*/);

    //install gpio isr service
    gpio_install_isr_service(ESP_INTR_FLAG_DEFAULT);
    //hook isr handler for specific gpio pin
    gpio_isr_handler_add(IR_SENSE, ir_sense_isr_handler, (void*) IR_SENSE);
    //hook isr handler for specific gpio pin
   // gpio_isr_handler_add(REED_SW, reed_sense_isr_handler, (void*) REED_SW);


    LED_ON;
    vTaskDelay(1000 / portTICK_RATE_MS);
    LED_OFF;

    ESP_ERROR_CHECK(button_init(&btnStart));

    FastLED.addLeds<LED_TYPE, DATA_PIN, GRB>(rgb_led, NUM_LEDS);
  //  rgb_led[2] = CRGB::Green;
  //  FastLED.show();

  
   xTaskCreate(detect_person, "detect_person", 2048, NULL, 10, NULL);
   xTaskCreatePinnedToCore(animate_rgb_leds, "animate_rgb_leds", 4096, NULL, 5, NULL, 0);

//    xTaskCreate(ultrasonic_test, "ultrasonic_test", 2048, NULL, 10, NULL);

    vTaskDelay(1000 / portTICK_RATE_MS);

    temp_data_queue = xQueueCreate(2, sizeof(mlx_temp_data_t));
    mutex_i2c = xSemaphoreCreateMutex();

   xTaskCreatePinnedToCore(display_on_oled, "display_on_oled", 8192, NULL, 9, NULL,1 );
   xTaskCreatePinnedToCore(mlx90614_get_temp, "mlx90614_get_temp", 4096, NULL, 11, &xGetTempNotifyTask_handle, 1);   
}

