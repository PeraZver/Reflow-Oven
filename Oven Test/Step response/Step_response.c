/* Pero 2016
*
*
* Step response 
* ATmega32u2
*
* The program measures the step response of the oven from MAX31855
* thermocouple amplifier via SPI. The oven is turned on by the PWM signal
* from A channel of the Timer1. PWM duty cycle is received from PC and stored in 
* EEPROM.
*
* The microcontroller communicates with the PC via LUFA interface. 
*
* Code adapted from Henrik Forsten
*
* April 2016
*/



#include "Step_response.h"
/* SPI ports on first 4 pins of PORTB (PB0..3) */
#define SS 0
#define SCK 1
#define MOSI 2
#define MISO 3

static volatile bool usb_connected = 0;

static uint16_t room_temp = 0;
static uint16_t duty_cycle = 0;
static uint16_t EEMEM eeprom_duty_cycle = 0;

static FILE USBSerialStream;

USB_ClassInfo_CDC_Device_t VirtualSerial_CDC_Interface =
	{
		.Config =
			{
				.ControlInterfaceNumber   = 0,
				.DataINEndpoint           =
					{
						.Address          = CDC_TX_EPADDR,
						.Size             = CDC_TXRX_EPSIZE,
						.Banks            = 1,
					},
				.DataOUTEndpoint =
					{
						.Address          = CDC_RX_EPADDR,
						.Size             = CDC_TXRX_EPSIZE,
						.Banks            = 1,
					},
				.NotificationEndpoint =
					{
						.Address          = CDC_NOTIFICATION_EPADDR,
						.Size             = CDC_NOTIFICATION_EPSIZE,
						.Banks            = 1,
					},
			},
	};


void usb_rx(void) {
	/*  Handle messages from host */
	char ReceivedChar;
	int ReceivedByte;
	/* Start commands with '!' */
	if ( (ReceivedChar = fgetc(&USBSerialStream)) != '!') {
		return;
	}
	/* Get the real command */
	while((ReceivedByte = fgetc(&USBSerialStream)) == EOF);
	ReceivedChar = (char)ReceivedByte;
	/* Choose duty cycle */
	if (ReceivedChar == 'W') {
		write_profile();
	}
	return;
}

void write_profile(void) {
	/* Read duty cycle from PC and replace current value in RAM and EEPROM */
	uint8_t settings[2];
	uint8_t i=0;
	int ReceivedChar;
	for(;;) {
		if ((ReceivedChar = fgetc(&USBSerialStream)) != EOF) {
			settings[i++] = (uint8_t)ReceivedChar;
			if (i==sizeof(duty_cycle))
			break;
		}
	}

	#define TOU16(x,k) ( (((uint16_t)x[k])<<8)|((uint16_t)x[k+1]) )

	/* Change the current settings in RAM */
	duty_cycle = TOU16(settings,0);
	
	fprintf(&USBSerialStream, "Unesena vrijednost je: %u", duty_cycle);
	fprintf(&USBSerialStream, "\n");
	fprintf(&USBSerialStream, "Set 0: %c", settings[0]);
	fprintf(&USBSerialStream, "\n");
	fprintf(&USBSerialStream, "Set 1: %c", settings[1]);
	fprintf(&USBSerialStream, "\n");
	OCR1A = duty_cycle;
	/* Write all settings as one block */
	eeprom_update_block(&duty_cycle, &eeprom_duty_cycle ,sizeof(duty_cycle));
}

void set_profile(void) {
	/* Read profile from EEPROM to RAM */
	eeprom_read_block(&duty_cycle, &eeprom_duty_cycle, sizeof(duty_cycle));
	OCR1A = duty_cycle;
}

uint16_t read_sensor(void) {
/* Bits:
 * 31 : sign,
 * 30 - 18 : thermocouple temperature,
 * 17 : reserved(0),
 * 16 : 1 if fault,
 * 15 - 4 : cold junction temperature,
 * 3 : reserved(0),
 * 2 : 1 if thermocouple is shorted to Vcc,
 * 1 : 1 if thermocouple is shorted to ground,
 * 0 : 1 if thermocouple is open circuit */

    /* Enable slave */
    uint8_t sensor[4];
    uint16_t temp;
    int8_t i;
    /* SS = 0 */
    PORTB = (0<<SS);

    /* Wait for the device */
    _NOP();
    _NOP();
    /* Transmit nothing */
    for(i=0;i<4;i++) {
        SPDR = 0x00;
        /* Wait for transmission to complete */
        while (!(SPSR & _BV(SPIF)));
        sensor[i] = SPDR;
    }

    /* Thermocouple temperature */
    if (sensor[0]&(1<<7)) {
        /* Negative temperature, clamp it to zero */
        temp = 0;
    } else {
        temp = (((uint16_t)sensor[0])<<6)+(sensor[1]>>2);
    }

    /* Room temperature */
    if (sensor[2]&(1<<7)) {
        /* Negative temperature, clamp it to zero */
        room_temp = 0;
    } else {
        room_temp = (((uint16_t)sensor[2])<<4)+(sensor[3]>>4);
        /* Sensor gives room temp as sixteenths of celsius,
         * divide it by four to get quarters of celsius. */
        room_temp = room_temp / 4;
    }

    if (sensor[1]&0x01) {
        /* Fault */
        fprintf(&USBSerialStream,"Fault:%u\n",sensor[3]&0b00000111);
    }

    /* Disable slave */
    PORTB = (1<<SS);
    return temp;
}

void setupHardware(void) {

    /* Disable wtachdog */
    MCUSR &= ~(1 << WDRF);
	wdt_disable();
    /* Disable prescaler */
	clock_prescale_set(clock_div_1);
	
	
	/******** Timer Setup **********/

    /* Set timer1 to count 1 second */
    TCNT1 = 0x0000;
    /* PWM output to channel A, pin PC6 */
    TCCR1A = 0b10000010;   // Output on Channel A (pin23, PC6), Fast PWM, COM1A1 = 10 - Clear OC1A on compare match, set at TOP
    /*  Set prescaler to divide by 256 for TMR1 */
    TCCR1B = 0b11011100;   // Fast PWM, CS1 = 100-prescale 256, together with TCCR1A, WGM is 1110 - Fast PWM, TOP is in ICR1, Update of OCR1A at TOP, 
    ICR1 = _ICR1;   // only Used for defining TOP Value: _ICR1 = 6250, presc=256 what means PWM frequnecy is 5Hz
    OCR1A = 0;       //  OFF value for the PWM, duty cycle is 625/6250 = 10%

    /* PC6 = Relay */
    DDRC   = 0b01000000;   // XTAL2 is on PC0, make it as input, Relay is on PC6, make it output
    PORTC  = 0x00;		   // Turn the relay down
    DDRD   = 0x00;         // Nothing, except UART which is unused.
	
	
	/******* SPI Setup **********/

    /* Enable SPI, Master, set clock rate fck/2 */
    SPCR = (1<<SPE) | (1<<MSTR);
    SPSR = (1<<SPI2X);
	/* Set !SS and SCK output, all others input */
    DDRB = (1<<SS)|(1<<SCK);
	PORTB |= (1<<SS);
	PORTB &= ~(1<<SCK);

 
 
	/******** USB Setup ***********/
	
    /* Initialize USB */
    USB_Init();
    CDC_Device_CreateStream(&VirtualSerial_CDC_Interface, &USBSerialStream);

    return;
}


int main(void) {


    bool tx_flag = 0;
    uint16_t temp = 0;
	
    setupHardware();
    set_profile();
	GlobalInterruptEnable();

    temp = read_sensor();

    while(1)
    {
        if (usb_connected) {
	        usb_rx();
            CDC_Device_USBTask(&VirtualSerial_CDC_Interface);
            USB_USBTask();
       }

        if (usb_connected && tx_flag) {
            tx_flag = 0;
            /* Send temp temperature */
			fprintf(&USBSerialStream, "temp:%u,room:%u,PWM:%u", (temp>>2), (room_temp>>2),OCR1A);
            fprintf(&USBSerialStream, "\n");
        }
		_delay_ms(250);
		/* Read the current temperature, updates temp and room_temp */
		temp = read_sensor();
		tx_flag = 1;
        
    }
}


/** Event handler for the library USB Connection event. */
void EVENT_USB_Device_Connect(void)
{
    usb_connected = 1;
}

/** Event handler for the library USB Disconnection event. */
void EVENT_USB_Device_Disconnect(void)
{
    usb_connected = 0;
}

/** Event handler for the library USB Configuration Changed event. */
void EVENT_USB_Device_ConfigurationChanged(void)
{
	CDC_Device_ConfigureEndpoints(&VirtualSerial_CDC_Interface);
}

/** Event handler for the library USB Control Request reception event. */
void EVENT_USB_Device_ControlRequest(void)
{
	CDC_Device_ProcessControlRequest(&VirtualSerial_CDC_Interface);
}

