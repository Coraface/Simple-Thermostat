#include <platform.h>
#include <stdio.h>
#include <gpio.h>
#include <lcd.h>
#include <leds.h>
#include <queue.h>
#include <timer.h>
#include "delay.h"

#define P_REC PC_7
#define P_TRIGGERED PB_6
#define P_ECHO PA_7
#define P_led_r PA_6
#define P_led_b PB_9                          
#define P_led_g PB_8

#define hot_temp 30
#define cool_temp 20
#define min_distance 15

int counter = 0;

// Sensor initializations to start communication
void ultrasonic_init() {
	gpio_set(P_TRIGGERED, 1);
	delay_us(12);
	gpio_set(P_TRIGGERED, 0);
}

uint8_t tempsensor_init() {
	uint8_t response = 0;
	gpio_set_mode(P_REC, Output);
	gpio_set(P_REC, 0);
	delay_us(480);
	
	gpio_set_mode(P_REC, Input);
	delay_us(80);
	if(!(gpio_get(P_REC))) response = 1;
	else response = 0;
	delay_us(400);
	return response;
}

// Write method
void tempsensor_write(uint8_t data) {
	gpio_set_mode(P_REC, Output);  				// Set pin as output
	for (int i = 0; i < 8; i++) {
		if ((data & (1<<i))!=0) {   				// If the bit is high write 1
			gpio_set_mode(P_REC, Output);  		
			gpio_set(P_REC, 0);  							// Pull pin low               
			delay_us(1); 
			
			gpio_set_mode(P_REC, Input);  		// Set pin as input
			delay_us(60);  								
		}
		else { 															// If the bit is low write 0
			gpio_set_mode(P_REC, Output);
			gpio_set(P_REC, 0);  						
			delay_us(60);  
			
			gpio_set_mode(P_REC, Input);
		}
	}
}

// Read method
uint8_t tempsensor_read() {
	uint8_t value = 0;
	gpio_set_mode(P_REC, Input);
	for (int i = 0; i < 8; i++) {
		gpio_set_mode(P_REC, Output);
		gpio_set(P_REC, 0);  								
		delay_us(2);  										

		gpio_set_mode(P_REC, Input);  		
		if(gpio_get(P_REC)) {  							// If the pin is HIGH read = 1
			value |= 1<<i;
		}
		delay_us(60); 
	}
	return value;
}

// Timer interrupt routine
void timer_isr() {
	counter++;
}

// Calculate average temperature
float calculate_avrg(Queue *q) {
	int i, sum = 0;
	for(i = 0; i < 25; i++) {
		sum += q->data[i];
	}
	return ((float)sum / 24);
}

// Calculate distance and check
void calculate_dist(char *rx_tmp, int temp, int avrg) {
	int distance = 0;											// Re-initialize in every loop
	int cycles = 0;												
	float duration = 0.0f;
	while(!(gpio_get(P_ECHO)));					
	while(gpio_get(P_ECHO)) {
		cycles += 33; 											// Check debugger 
	}
	duration = (float) cycles / SystemCoreClock;
	distance = duration * 170 * 100;   		// Measured distance in cm
	
	if(distance < min_distance) {
		lcd_print("Current oC:");						// Print current temperature
		sprintf(rx_tmp, " %i  ", (int)temp);
		lcd_print(rx_tmp);
		
		lcd_set_cursor(0,1);
		lcd_print("Average oC:");						// Print average temperature
		sprintf(rx_tmp, " %i  ", (int)avrg);				
		lcd_print(rx_tmp);
	}
}

// Checks temperature
void check_tmp_dist(int temp) {
	if(temp >= hot_temp) {
		gpio_set(P_led_r, 1);
		gpio_set(P_led_g, 1);						// A switch turns on (led in this case)
		
		lcd_clear();
		lcd_set_cursor(0,0);
		lcd_print("  Hot Hot Hot!");
	}
	else if(temp < cool_temp) {
		gpio_set(P_led_b, 1);
		
		lcd_clear();
		lcd_set_cursor(0,0);
		lcd_print(" Is it snowing?");
	}
	else {
		gpio_set(P_led_r, 0);
		gpio_set(P_led_g, 0);
		gpio_set(P_led_b, 0);

		lcd_print("Calculating avrg");
		lcd_set_cursor(0,1);
		lcd_print("temperature...");
	}
}

// Main
int main() {
	
	// Variables
	Queue rx_queue;												// Queue to hold data
	
	char rx_tmp[16];											// Holds the string to be displayed
	volatile uint8_t sres = 0, qres = 0;	// Sensor and queue response for debugging
	uint8_t tmp_byte1, tmp_byte2, temp = 0;
	volatile float avrg;
	
	// Gpios
	gpio_set_mode(P_TRIGGERED, Output);
	gpio_set_mode(P_ECHO, Input);
	gpio_set_mode(P_led_r, Output);
	gpio_set_mode(P_led_g, Output);
	gpio_set_mode(P_led_b, Output);

	// Inits
	leds_init();
	lcd_init();
	queue_init(&rx_queue, 25);
	timer_init(1000000);
	timer_set_callback(timer_isr);
	__enable_irq();

	lcd_print("Initialising...");
 	delay_ms(2000);
	lcd_clear();

	timer_enable();
	while(1) {
		leds_set(1);
		lcd_set_cursor(0,0);
		
		// Start getting signal from ultrasonic sensor
		ultrasonic_init();
		calculate_dist(rx_tmp,temp, avrg);
		
		// Start getting signal from temperature sensor
		if(counter == 5) {
			counter = 0;
			leds_set(0);
			
			sres = tempsensor_init();
			delay_ms(1);
			tempsensor_write(0xCC);  						// Skip ROM
			tempsensor_write(0x44);  						// Convert t
			delay_ms(200);
			
			sres = tempsensor_init();
			tempsensor_write(0xCC);  						
			tempsensor_write(0xBE);  						// Read Scratch-pad
		
			tmp_byte1 = tempsensor_read();
			tmp_byte2 = tempsensor_read();
			temp = (float)((tmp_byte2<<8)|tmp_byte1)/16; // Measured temperature
			
			// If queue is full, get average, else enqueue and check temperature
			if(queue_is_full(&rx_queue)) {
				__disable_irq();
				avrg = calculate_avrg(&rx_queue);
				rx_queue.tail = 0;								// Reset the queue to store temps for the next 2 minutes
				qres = queue_enqueue(&rx_queue, temp); 	// Store the 1st value of next 2 minutes
				
				lcd_clear();
				lcd_set_cursor(0,0);
				lcd_print("  Average temp");
				lcd_set_cursor(0,1);

				sprintf(rx_tmp, "     %ioC     ", (int)avrg);
				lcd_print(rx_tmp);
				delay_ms(10000);
				__enable_irq();
			}
			else {
				qres = queue_enqueue(&rx_queue, temp);				
				check_tmp_dist(temp);
			}
		}
	}
}