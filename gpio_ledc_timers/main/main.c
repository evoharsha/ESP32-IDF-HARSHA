#include <stdio.h>
#include "freertos/FreeRTOS.h"
#include "freertos/task.h"
#include "driver/ledc.h"
#include "esp_err.h"
#include <string.h>
#include <stdlib.h>
#include "freertos/queue.h"
#include "driver/gpio.h"

#define LEDC_HS_TIMER          LEDC_TIMER_0
#define LEDC_HS_MODE           LEDC_HIGH_SPEED_MODE
#define LEDC_HS_CH0_GPIO       (2)
#define LEDC_HS_CH0_CHANNEL    LEDC_CHANNEL_0

#define LEDC_TEST_DUTY         (7000)
#define LEDC_TEST_FADE_TIME    (5000)
/**
 * GPIO status:
 * GPIO18: output
 * GPIO19: output
 * GPIO4:  input, pulled up, interrupt from rising edge and falling edge
 * GPIO5:  input, pulled up, interrupt from rising edge.
 *
 * Test:
 * Connect GPIO18 with GPIO4
 * Connect GPIO19 with GPIO5
 * Generate pulses on GPIO18/19, that triggers interrupt on GPIO4/5
 **/


#define GPIO_OUTPUT_IO_0    18
#define GPIO_OUTPUT_IO_1    19// | BITWISE OR
#define GPIO_OUTPUT_PIN_SEL  ((1ULL<<GPIO_OUTPUT_IO_0) | (1ULL<<GPIO_OUTPUT_IO_1))//unsigned long long - ULL
#define GPIO_INPUT_IO_0     4
#define GPIO_INPUT_IO_1     5
#define GPIO_INPUT_PIN_SEL  ((1ULL<<GPIO_INPUT_IO_0) | (1ULL<<GPIO_INPUT_IO_1))
#define ESP_INTR_FLAG_DEFAULT 0

static xQueueHandle gpio_evt_queue = NULL;

static void IRAM_ATTR gpio_isr_handler(void* arg) //forces data into IRAM
{
    uint32_t gpio_num = (uint32_t) arg;
    xQueueSendFromISR(gpio_evt_queue, &gpio_num, NULL);
}

static void gpio_task_example(void* arg)
{
    uint32_t io_num;
    for(;;) {
        if(xQueueReceive(gpio_evt_queue, &io_num, portMAX_DELAY)) {
            printf("GPIO[%d] intr, val: %d\n", io_num, gpio_get_level(io_num));
        }
    }
}




void app_main(void)
{

	//Configure Timer
	ledc_timer_config_t ledc_timer = {
			.speed_mode = LEDC_HS_MODE,
			.duty_resolution = LEDC_TIMER_13_BIT,
			.timer_num = LEDC_HS_TIMER,
			.freq_hz = 5000,
			.clk_cfg = LEDC_AUTO_CLK,
	};

	// Set configuration of timer0 for high speed channels
	ledc_timer_config(&ledc_timer);

	//configure channel
	ledc_channel_config_t ledc_channel ={
	           .channel    = LEDC_HS_CH0_CHANNEL,
	           .duty       = 0,
	           .gpio_num   = LEDC_HS_CH0_GPIO,
	           .speed_mode = LEDC_HS_MODE,
	           .hpoint     = 0,
	           .timer_sel  = LEDC_HS_TIMER
	};

	ledc_channel_config(&ledc_channel);

	// Initialize fade service.
	ledc_fade_func_install(0);


	gpio_config_t io_conf;
	    //disable interrupt
	    io_conf.intr_type = GPIO_PIN_INTR_DISABLE;
	    //set as output mode
	    io_conf.mode = GPIO_MODE_OUTPUT;
	    //bit mask of the pins that you want to set,e.g.GPIO18/19
	    io_conf.pin_bit_mask = GPIO_OUTPUT_PIN_SEL;
	    //disable pull-down mode
	    io_conf.pull_down_en = 0;
	    //disable pull-up mode
	    io_conf.pull_up_en = 0;
	    //configure GPIO with the given settings
	    gpio_config(&io_conf);


	    //interrupt of rising edge
	     io_conf.intr_type = GPIO_PIN_INTR_POSEDGE;
	     //bit mask of the pins, use GPIO4/5 here
	     io_conf.pin_bit_mask = GPIO_INPUT_PIN_SEL;
	     //set as input mode
	     io_conf.mode = GPIO_MODE_INPUT;
	     //enable pull-up mode
	     io_conf.pull_up_en = 1;
	     gpio_config(&io_conf);

	     //change gpio intrrupt type for one pin
	       gpio_set_intr_type(GPIO_INPUT_IO_0, GPIO_INTR_ANYEDGE);

	       //create a queue to handle gpio event from isr
	       gpio_evt_queue = xQueueCreate(10, sizeof(uint32_t));
	       //start gpio task
	       xTaskCreate(gpio_task_example, "gpio_task_example", 2048, NULL, 10, NULL);

	       //install gpio isr service
	       gpio_install_isr_service(ESP_INTR_FLAG_DEFAULT);
	       //hook isr handler for specific gpio pin
	       gpio_isr_handler_add(GPIO_INPUT_IO_0, gpio_isr_handler, (void*) GPIO_INPUT_IO_0);
	       //hook isr handler for specific gpio pin
	       gpio_isr_handler_add(GPIO_INPUT_IO_1, gpio_isr_handler, (void*) GPIO_INPUT_IO_1);

	       //remove isr handler for gpio number.
	       gpio_isr_handler_remove(GPIO_INPUT_IO_0);
	       //hook isr handler for specific gpio pin again
	       gpio_isr_handler_add(GPIO_INPUT_IO_0, gpio_isr_handler, (void*) GPIO_INPUT_IO_0);

	       int cnt = 0;
	       while(1) {
	           printf("cnt: %d\n", cnt++);
	           vTaskDelay(1000 / portTICK_RATE_MS);
	           gpio_set_level(GPIO_OUTPUT_IO_0, cnt % 2);
	           gpio_set_level(GPIO_OUTPUT_IO_1, cnt % 2);

	   		//Change duty cycle using H/W
	   		printf("1. LEDC fade up to duty = %d\n", LEDC_TEST_DUTY);
	   		ledc_set_fade_with_time(ledc_channel.speed_mode, ledc_channel.channel, LEDC_TEST_DUTY, LEDC_TEST_FADE_TIME);
	   		ledc_fade_start(ledc_channel.speed_mode, ledc_channel.channel, LEDC_FADE_NO_WAIT);
	   		vTaskDelay(LEDC_TEST_FADE_TIME / portTICK_PERIOD_MS);

	   		printf("2. LEDC fade down to duty = 0\n");
	   		ledc_set_fade_with_time(ledc_channel.speed_mode, ledc_channel.channel, 0, LEDC_TEST_FADE_TIME);
	   		ledc_fade_start(ledc_channel.speed_mode, ledc_channel.channel, LEDC_FADE_NO_WAIT);
	   		vTaskDelay(LEDC_TEST_FADE_TIME / portTICK_PERIOD_MS);
	       }
}

