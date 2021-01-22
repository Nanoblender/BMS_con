/*
 * This program is reusing code from the  pill duck project (https://github.com/satoshinm/pill_duck)
 *  
 * This program is free software: you can redistribute it and/or modify
 * it under the terms of the GNU Lesser General Public License as published by
 * the Free Software Foundation, either version 3 of the License, or
 * (at your option) any later version.
 *
 * This library is distributed in the hope that it will be useful,
 * but WITHOUT ANY WARRANTY; without even the implied warranty of
 * MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 * GNU Lesser General Public License for more details.
 *
 * You should have received a copy of the GNU Lesser General Public License
 * along with this library.  If not, see <http://www.gnu.org/licenses/>.
 */


#include <libopencm3/cm3/nvic.h>
#include <libopencm3/cm3/systick.h>

#include <libopencm3/usb/usbd.h>
#include <libopencm3/usb/hid.h>
#include <libopencm3/cm3/scb.h>
#include <libopencm3/stm32/desig.h>
#include <libopencm3/stm32/rcc.h>
#include <libopencm3/stm32/gpio.h>
#include <libopencm3/stm32/adc.h>

#include "hid.h"
#include "encoder.h"

//Definition of the input used for the buttons, turntable's pin are defined in encode.h
#define B1_PORT GPIOG
#define B1_PIN GPIO9

#define B2_PORT GPIOG
#define B2_PIN GPIO14

#define B3_PORT GPIOF
#define B3_PIN GPIO15

#define B4_PORT GPIOE
#define B4_PIN GPIO13

#define B5_PORT GPIOF
#define B5_PIN GPIO14

#define B6_PORT GPIOE
#define B6_PIN GPIO11

#define B7_PORT GPIOE
#define B7_PIN GPIO9

#define B8_PORT GPIOE
#define B8_PIN GPIO8

#define B9_PORT GPIOE
#define B9_PIN GPIO7

//How many switch are connected
#define AMOUNT_SWITCHS 9

//Joystick buttons used as turntable up and down when the truntable is in digital mode
#define TURNTABLE_UP 9
#define TURNTABLE_DOWN 10
//Number of step of the turntable encoder necessary to send a button press
#define TURNTABLE_TRESHOLD 4

//frequency of systick interrupt
#define SYSTICK_FREQ 100000

//Period during which switches's signal must remain stable in ms 
#define DEBOUNCE_TIME 10

//Time enlapsed since startup in systick
volatile uint32_t system_millis;
//Define if the turtable send an analog value or buttons press
uint8_t analog_mode=1;

static usbd_device *usbd_dev;


const struct usb_device_descriptor dev_descr = {
	.bLength = USB_DT_DEVICE_SIZE,
	.bDescriptorType = USB_DT_DEVICE,
	.bcdUSB = 0x0200,
	.bDeviceClass = 0,
	.bDeviceSubClass = 0,
	.bDeviceProtocol = 0,
	.bMaxPacketSize0 = 64,
	.idVendor = 0x0483,
	.idProduct = 0x5710,
	.bcdDevice = 0x0100,
	.iManufacturer = 1,
	.iProduct = 2,
	.iSerialNumber = 3,
	.bNumConfigurations = 1,
};

const struct usb_interface ifaces[] = {{
	.num_altsetting = 1,
	.altsetting = &hid_iface,
}};

const struct usb_config_descriptor config = {
	.bLength = USB_DT_CONFIGURATION_SIZE,
	.bDescriptorType = USB_DT_CONFIGURATION,
	.wTotalLength = 0,
	.bNumInterfaces = sizeof(ifaces)/sizeof(ifaces[0]),
	.bConfigurationValue = 1,
	.iConfiguration = 0,
	.bmAttributes = 0xa0,//Bus Powered Remote Wakeup
	.bMaxPower = 0xF0,

	.interface = ifaces,
};

static const char *usb_strings[] = {
	"Nanoblender",
	"BMS controller",
	"TEST",
};

void sys_tick_handler(void)
{
	system_millis++;
}

void send_gamepad(struct gamepad_report_t report, uint8_t * new_report){
	/** Send the gamepad's HID report
	 **/
	uint16_t bytes_written = 0;
	bytes_written = usbd_ep_write_packet(usbd_dev, 0x81, &report, GAMEPAD_REPORT_SIZE);
	if(bytes_written!=0) *new_report=0;
}

void button_scan(struct gamepad_report_t * report_p, uint8_t * new_report_p){
	/**Scan the switches and update the buttons if there is a change
	 * input: hid report that contains the jostick's state 
	**/
	static uint8_t buttons_in[16]={0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
	static uint8_t buttons[16]={0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
	static uint32_t press_time[16]={0,0,0,0,0,0,0,0,0,0,0,0,0,0,0,0};
	uint8_t i=0;
	uint32_t current_time =0;
	buttons_in[0]=!(gpio_get(B1_PORT, B1_PIN)==B1_PIN);
	buttons_in[1]=!(gpio_get(B2_PORT, B2_PIN)==B2_PIN);
	buttons_in[2]=!(gpio_get(B3_PORT, B3_PIN)==B3_PIN);
	buttons_in[3]=!(gpio_get(B4_PORT, B4_PIN)==B4_PIN);
	buttons_in[4]=!(gpio_get(B5_PORT, B5_PIN)==B5_PIN);
	buttons_in[5]=!(gpio_get(B6_PORT, B6_PIN)==B6_PIN);
	buttons_in[6]=!(gpio_get(B7_PORT, B7_PIN)==B7_PIN);
	buttons_in[7]=!(gpio_get(B8_PORT, B8_PIN)==B8_PIN);
	buttons_in[8]=!(gpio_get(B9_PORT, B9_PIN)==B9_PIN);
	current_time = system_millis;
	for(i=0;i<AMOUNT_SWITCHS;i++){
		//Debouncing and updating buttons' state
		if(buttons_in[i]==buttons[i])press_time[i]=current_time;
		if((current_time-press_time[i])>DEBOUNCE_TIME*(SYSTICK_FREQ/1000)){
			buttons[i]=buttons_in[i];
			report_p->buttons = (report_p->buttons & ~(1<<i)) | ((buttons[i]<<i) & (1<<i));
			*new_report_p=1;
		}
	}
}

void turntable_scan(struct gamepad_report_t * report_p, uint8_t * new_report_p){
	/**Check the postiton of the encoder and update the reportif necessary
	 * input: hid report that contains the jostick's state 
	**/
	static uint8_t old_enc_val=0;
	static uint8_t enc_val=0;
	static uint32_t tt_press=0;
	int8_t enc_diff=0;
	uint8_t tt_bt=0;
	uint8_t down_limit=(ENCODER_PERIOD)>>2;
	uint8_t up_limit=(ENCODER_PERIOD)-((ENCODER_PERIOD)>>2);
	enc_val=encoder_get_counter();
	if(analog_mode){
		if(old_enc_val!=enc_val){
			report_p->x=enc_val;
			*new_report_p=1;
			old_enc_val=enc_val;
		}
	}
	else{
		if(old_enc_val!=enc_val){
			if((enc_val<down_limit) & (old_enc_val>up_limit))enc_diff=enc_val+ENCODER_PERIOD-old_enc_val;
			else if ((enc_val>up_limit) & (old_enc_val<down_limit))enc_diff=enc_val-ENCODER_PERIOD-old_enc_val;
			else enc_diff=enc_val-old_enc_val;
			if(enc_diff>TURNTABLE_TRESHOLD)tt_bt=TURNTABLE_UP;
			else if(enc_diff<-TURNTABLE_TRESHOLD) tt_bt=TURNTABLE_DOWN;
			if(tt_bt>0){
				old_enc_val=enc_val;
				report_p->buttons = report_p->buttons | (1<<tt_bt);
				*new_report_p=1;
				tt_press=system_millis;
			}
		}
		if(((system_millis-tt_press)>5000) & ((report_p->buttons & ((1<<TURNTABLE_UP)+(1<<TURNTABLE_DOWN)))>0)){
			report_p->buttons = report_p->buttons & ~((1<<TURNTABLE_UP)+(1<<TURNTABLE_DOWN));
			*new_report_p=1;
		}
	}
}

static void setup_clock(void) {
	rcc_clock_setup_pll(&rcc_hse_8mhz_3v3[RCC_CLOCK_3V3_168MHZ]);
	/*Enable GPIO clock*/
	rcc_periph_clock_enable(RCC_GPIOA);
	rcc_periph_clock_enable(RCC_GPIOB);
	rcc_periph_clock_enable(RCC_GPIOC);
	rcc_periph_clock_enable(RCC_GPIOE);
	rcc_periph_clock_enable(RCC_GPIOF);
	rcc_periph_clock_enable(RCC_GPIOG);
	/* Enable USB clock*/
	rcc_periph_clock_enable(RCC_OTGFS);
	/**systick setup
	  * 168000000/1680=100000 systick clock is 100kHz (0.01ms between each interrupt) 
	  **/
	systick_set_reload((168000000/SYSTICK_FREQ)-1);
	systick_set_clocksource(STK_CSR_CLKSOURCE_AHB);
	systick_counter_enable();
	/* this done last */
	systick_interrupt_enable();
}

static void setup_gpio(void) {
	gpio_mode_setup(B1_PORT, GPIO_MODE_INPUT, GPIO_PUPD_PULLUP, B1_PIN);
	gpio_mode_setup(B2_PORT, GPIO_MODE_INPUT, GPIO_PUPD_PULLUP, B2_PIN);
	gpio_mode_setup(B3_PORT, GPIO_MODE_INPUT, GPIO_PUPD_PULLUP, B3_PIN);
	gpio_mode_setup(B4_PORT, GPIO_MODE_INPUT, GPIO_PUPD_PULLUP, B4_PIN);
	gpio_mode_setup(B5_PORT, GPIO_MODE_INPUT, GPIO_PUPD_PULLUP, B5_PIN);
	gpio_mode_setup(B6_PORT, GPIO_MODE_INPUT, GPIO_PUPD_PULLUP, B6_PIN);
	gpio_mode_setup(B7_PORT, GPIO_MODE_INPUT, GPIO_PUPD_PULLUP, B7_PIN);
	gpio_mode_setup(B8_PORT, GPIO_MODE_INPUT, GPIO_PUPD_PULLUP, B8_PIN);
	gpio_mode_setup(B9_PORT, GPIO_MODE_INPUT, GPIO_PUPD_PULLUP, B9_PIN);
	gpio_mode_setup(GPIOB, GPIO_MODE_OUTPUT,GPIO_PUPD_NONE, GPIO0);
	gpio_mode_setup(GPIOG, GPIO_MODE_OUTPUT,GPIO_PUPD_NONE, GPIO6);

}

/* Buffer to be used for control requests. */
uint8_t usbd_control_buffer[128];

int main(void)
{
	uint32_t woke=0;
	struct gamepad_report_t report={0,0};
	uint8_t new_report=0;
	setup_clock();
	setup_gpio();
	encoder_setup();
	/* USB pins */
	gpio_mode_setup(GPIOA, GPIO_MODE_AF, GPIO_PUPD_NONE, GPIO11 | GPIO12);
	gpio_set_af(GPIOA, GPIO_AF10, GPIO11 | GPIO12);
	/*USB setup*/
	usbd_dev = usbd_init(&otgfs_usb_driver, &dev_descr, &config, usb_strings,sizeof(usb_strings)/sizeof(char *),usbd_control_buffer, sizeof(usbd_control_buffer));
	usbd_register_set_config_callback(usbd_dev, hid_set_config);
	gpio_clear(GPIOG,GPIO6);
	/*Wait  for the usb to setup*/
	woke = system_millis + 50000;
	while (woke > system_millis){
		//Checking if Button8 is held down to set the turntable in digital mode
		while(gpio_get(B8_PORT, B8_PIN)!=B8_PIN){
			analog_mode=0;
		}
	}
	

	while (1){
		usbd_poll(usbd_dev);
		button_scan(&report, &new_report);
		turntable_scan(&report, &new_report);
		if(new_report)send_gamepad(report, &new_report);
	}
}