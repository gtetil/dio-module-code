#include <avr/io.h>
#include <avr/sfr_defs.h>
#include <avr/sleep.h>
#include <avr/power.h>
#include <util/delay.h>
#include <mcp_can.h>
#include <SPI.h>
#include <avr/interrupt.h>

//CAN variables
long unsigned int rxId;
unsigned char len = 0;
unsigned char rxBuf[8];
unsigned char flagRecv = 0;
const int SPI_CS_PIN = 10;
#define CAN_INT 2                                            // Set INT to pin 2
MCP_CAN CAN(SPI_CS_PIN);                                    // Set CS pin
byte message[8] = {0, 0, 0, 0, 0, 0, 0, 0};
unsigned long can_filter = 0x00100000; //nibble 5 is to identify this is for the output message (input from RPi)
unsigned long can_filter_address = 0;
int sent_msg;
uint8_t do_data;
// To use the sleep mode of the transceiver (MCP2551), it's Rs pin must be connected to either the MCP2515 or 
// any free Arduino output.
#define RS_TO_MCP2515 true                                   // Set this to false if Rs is connected to your Arduino
#define RS_OUTPUT MCP_RX0BF                                  // RX0BF is a pin of the MCP2515. You can also define an Arduino pin here
#define KEEP_AWAKE_TIME 3000                                  // time the controller will stay awake after the last activity on the bus (in ms)
unsigned long lastBusActivity = millis();

//CAN addresses
uint16_t CAN_input_address;
uint16_t CAN_output_address;

static inline void initOutputs(void) {
	
	DDRB |= (1 << DDB0);  //set PB0 as output DO0
	DDRB |= (1 << DDB1);  //set PB1 as output DO1
	DDRD |= (1 << DDD3);  //set PD3 as output DO2
	DDRD |= (1 << DDD4);  //set PD4 as output DO3
	DDRD |= (1 << DDD5);  //set PD5 as output DO4
	DDRD |= (1 << DDD6);  //set PD6 as output DO5
	
}

static inline void initInputs(void) {
	
	DDRC &= ~(1 << PINC0);  //set PINB0 to input, DI0
	DDRC &= ~(1 << PINC1);  //set PINB1 to input, DI1
	DDRC &= ~(1 << PINC2);  //set PINB2 to input, DI2
	DDRC &= ~(1 << PINC3);  //set PINB3 to input, DI3
	DDRC &= ~(1 << PINC4);  //set PINB4 to input, DI4
	DDRC &= ~(1 << PINC5);  //set PINB5 to input, DI5

}

static inline void initADC(void) {
	
	ADCSRA |= (1 << ADPS0);       /* ADC clock prescaler /128 */
	ADCSRA |= (1 << ADPS1);       /* ADC clock prescaler /128 */
	ADCSRA |= (1 << ADPS2);       /* ADC clock prescaler /128 */
	ADCSRA |= (1 << ADEN);        /* enable ADC */
	
}

uint8_t read_ADC_as_DI(uint8_t channel) {
	
	ADMUX = (1 << REFS0) | channel;  // set reference AVCC = 5V
	ADCSRA |= (1<<ADSC);         // start conversion
	loop_until_bit_is_clear(ADCSRA,ADSC); // wait for conversion complete
	if (ADC > 512) {  // >2.5V
		return 1;
	}
	else {
		return 0;
	}
}

static inline void initCANaddresses(void) {
	//create CAN addresses
	DDRD &= ~(1 << PIND7);  // set PIND7 to input, for CAN ADDR_0
	PORTD |= (1 << PIND7);  // set PIND7 pull-up resistor
	DDRD &= ~(1 << PIND0);  // set PIND0 to input, for CAN ADDR_2
	PORTD |= (1 << PIND0);  // set PIND0 pull-up resistor
	uint8_t adc7_as_di = read_ADC_as_DI(7);  // CAN_ADDR_1
	uint8_t base_address = ((~PIND & 0b10000000) >> 7) | ((~PIND & 0b00000001) << 2) | (adc7_as_di << 1);
	CAN_input_address = base_address;
	CAN_output_address = 0x010 | base_address;
}

static inline void set_output(uint8_t do_data, uint8_t ch_from_can, uint8_t port, uint8_t pin) {
	
	if (bit_is_set(do_data, ch_from_can)) {
		if (port == 0) {  // port 0 represents PORTB
			PORTB |= (1 << pin);  // turn on output
		}
		else {  // anything else represents PORTD
			PORTD |= (1 << pin);  // turn on output
		}
	}
	else {
		if (port == 0) {
			PORTB &= ~(1 << pin);  // turn off output
		}
		else {
			PORTD &= ~(1 << pin);  // turn off output
		}
	}
}

static inline void set_digital_outputs(uint8_t do_data) {
	
	set_output(do_data, 0, 0, PINB0); // set output PB0, DO0
	set_output(do_data, 1, 0, PINB1); // set output PB1, DO1
	set_output(do_data, 2, 1, PIND3); // set output PD3, DO2
	set_output(do_data, 3, 1, PIND4); // set output PD4, DO3
	set_output(do_data, 4, 1, PIND5); // set output PD5, DO4
	set_output(do_data, 5, 1, PIND6); // set output PD6, DO5
	
}

void MCP2515_ISR()
{
	flagRecv = 1;
}

void setup() {
	
	// -------- Inits --------- //
	initOutputs();
	initInputs();
	initADC();
	initCANaddresses();
	
	//CAN setup
	CAN.begin(CAN_500KBPS, MCP_16MHz);
	// attach interrupt
	pinMode(CAN_INT, INPUT);
	attachInterrupt(digitalPinToInterrupt(CAN_INT), MCP2515_ISR, FALLING);
	CAN.setSleepWakeup(1);                                   // this tells the MCP2515 to wake up on incoming messages
	
	// Pull the Rs pin of the MCP2551 transceiver low to enable it:
    if(RS_TO_MCP2515) 
    {
      CAN.mcpPinMode(MCP_RX0BF, MCP_PIN_OUT);
      CAN.mcpDigitalWrite(RS_OUTPUT, LOW);
    } else {
      pinMode(RS_OUTPUT, OUTPUT);
      digitalWrite(RS_OUTPUT, LOW);
    }
	
}


void loop() {  // must use this Arduino loop() and setup() for CAN to work.  It must have something to do with initializing libraries.
	
	uint8_t ignition;
	uint8_t prev_ign;
	
	while(1) {
		
		ignition = read_ADC_as_DI(6);
		//ignition = (~PINC & 0b00000001);  // used for debug when ADC6 is not available (on DIP version of ATMEGA328P)
		
		if ((ignition == 0) && (prev_ign == 1)) {  // if ignition was on and now off, turn off all digital outputs
			set_digital_outputs(0);
		}
		prev_ign = ignition;
		
		if (flagRecv) {
			
			flagRecv = 0;                   // clear flag
			lastBusActivity = millis();
			
			while (CAN_MSGAVAIL == CAN.checkReceive()) {
				CAN.readMsgBuf(&len, rxBuf); // Read data: len = data length, buf = data byte(s)
				do_data = rxBuf[7];  // Digital output data is contained in byte 7
				if (ignition == 0) {  // if ignition is zero, then turn off all digital outputs  // THIS NEEDS TO CHANGE TO ADC6!!!!
					do_data = 0;
				}
				set_digital_outputs(do_data);
			}
			
		}
		//else if ((ignition == 0) && (prev_ign == 1)) {  // if ignition was on and now off, turn off all digital outputs
		else if(millis() > lastBusActivity + KEEP_AWAKE_TIME) {
			
			set_digital_outputs(0);
			CAN.sleep();
			
			// Put the transceiver into standby (by pulling Rs high):
			if (RS_TO_MCP2515) {
				CAN.mcpDigitalWrite(RS_OUTPUT, HIGH);
			}
			else {
				digitalWrite(RS_OUTPUT, HIGH);
			}
			
			cli(); // Disable interrupts
			if(!flagRecv) // Make sure we havn't missed an interrupt between the check above and now. If an interrupt happens between now and sei()/sleep_cpu() then sleep_cpu() will immediately wake up again
			{
				set_sleep_mode(SLEEP_MODE_PWR_DOWN);
				sleep_enable();
				sleep_bod_disable();
				sei();
				sleep_cpu();
				// Now the Arduino sleeps until the next message arrives...
				sleep_disable();
			}
			sei();
			CAN.wake();
			
			// Wake up the transceiver:
			if(RS_TO_MCP2515) {
				CAN.mcpDigitalWrite(RS_OUTPUT, LOW);
			}
			else {
				digitalWrite(RS_OUTPUT, LOW);
			} 
			
		}
			
		message[0] = (~PINC & 0b00111111) | (ignition << 6);  // put digital inputs into CAN message
		
		cli();
		sent_msg = CAN.sendMsgBuf(CAN_output_address, 0, 8, message);  //id, standard frame, data len, data bu
		sei();
		
		_delay_ms(100);
		
	}
	
}

