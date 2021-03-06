/*
 * OpenTherm encoder, by AMVV
 * based on a work by: Sebastian Wallin
 * description:
 * Example on opentherm communication
 * This example will set the water temperature to 38 degrees
 *
 * This OT encoder implements a manchester encoder, which transmits a bit every 1ms,
 * so it fires every 0.5ms to make a transition.
 * it is not very clean code...
 */

#define OUTPUT_PIN (6)//for use with JeeNodeUSB

/* Timer2 reload value, globally available */
unsigned int tcnt2;

/*my stuff amvv */
int cycles = 0;
int originalvalue = 0;
int y;
int bits_sent = 0;

byte bit_out = HIGH;
boolean done = true;
byte state = 0; //0 - start bit; 1...4 - message: 5 - stop bit

//not yet used...
typedef struct {
	byte message_type;
	byte data_id;
	byte data_value_h;
	byte data_value_l;
}
OTdata;


//OTdata MM;
byte MM1 = 0;
byte MM2 = 0;
byte MM3 = 0;
byte MM4 = 0;

/* Setup phase: configure and enable timer2 overflow interrupt */
void setup()
{

	Serial.begin(19200);
	/* Configure the test pin as output */
	pinMode(OUTPUT_PIN, OUTPUT);

	StopInterrupts();

	StartInterrupts();
}

void StopInterrupts()
{
	TIMSK2 &= ~(1 << TOIE2);
}

void StartInterrupts()
{
	/* Configure timer2 in normal mode (pure counting, no PWM etc.) */
	TCCR2A &= ~((1 << WGM21) | (1 << WGM20));
	TCCR2B &= ~(1 << WGM22);

	/* Select clock source: internal I/O clock */
	ASSR &= ~(1 << AS2);

	/* Disable Compare Match A interrupt enable (only want overflow) */
	TIMSK2 &= ~(1 << OCIE2A);

	/* Now configure the prescaler to CPU clock divided by 128 */
	TCCR2B |= (1 << CS22)  | (1 << CS20); // Set bits
	TCCR2B &= ~(1 << CS21);           // Clear bit

	/* We need to calculate a proper value to load the timer counter.
	 * The following loads the value 131 into the Timer 2 counter register
	 * The math behind this is:
	 * (CPU frequency) / (prescaler value) = 125000 Hz = 8us.
	 * (desired period) / 8us = 125.
	 * MAX(uint8) + 1 - 125 = 131;
	 */
	/* Save value globally for later reload in ISR */
	tcnt2 = 192;

	/* Finally load end enable the timer */
	TCNT2 = tcnt2;
	TIMSK2 |= (1 << TOIE2);
}

boolean logic_transition = false;

/*
 * Install the Interrupt Service Routine (ISR) for Timer2 overflow.
 * This is normally done by writing the address of the ISR in the
 * interrupt vector table but conveniently done by using ISR()  */
ISR(TIMER2_OVF_vect)
{
	/* Reload the timer */
	TCNT2 = tcnt2;

	switch (state) {
	case 0: //start bit
		if (logic_transition == false) {
			y = 1;
			logic_transition = true;
			//Serial.print(y);
		} else {
			y = !y;
			logic_transition = false;
			state++;
		}
		break;

	case 1://message byte 1
		if (logic_transition == false) {
			y = ((MM1) & 128) >> 7;
			MM1 = MM1 << 1;
			logic_transition = true;
			//Serial.print(y);
		} else {
			y = !y;

			logic_transition = false;
		}
		break;
	case 2://message byte 2
		if (logic_transition == false) {
			y = ((MM2) & 128) >> 7;
			MM2 = MM2 << 1;
			logic_transition = true;
			//Serial.print(y);
		} else {
			y = !y;

			logic_transition = false;
		}
		break;
	case 3://message byte 3
		if (logic_transition == false) {
			y = ((MM3) & 128) >> 7;
			MM3 = MM3 << 1;
			logic_transition = true;
			//Serial.print(y);
		} else {
			y = !y;

			logic_transition = false;
		}
		break;
	case 4://message byte 3
		if (logic_transition == false) {
			y = ((MM4) & 128) >> 7;
			MM4 = MM4 << 1;
			logic_transition = true;
			//Serial.print(y);
		} else {
			y = !y;

			logic_transition = false;
		}
		break;
	case 5: //stop bit
		if (logic_transition == false) {
			y = 1;
			logic_transition = true;
			//Serial.println(y);
		} else {
			y = !y;
			logic_transition = false;
			state = 0;
		}
		break;
	}


	digitalWrite(OUTPUT_PIN, !y);
	if (logic_transition == true)
		Serial.print(y, DEC);

	bits_sent = bits_sent + 1;

	if (bits_sent == 18) { //(sizeof(MM*3)*8*2 + 2))  MM1 sent
		state++;
	}
	if (bits_sent == 34) { //(sizeof(MM*3)*8*2 + 2))  MM2 sent
		state++;
	}
	if (bits_sent == 50) { //(sizeof(MM*3)*8*2 + 2))  MM3 sent
		state++;
	}
	if (bits_sent == 66) { //(sizeof(MM*3)*8*2 + 2)) MM4 sent
		state++;
	}
	if (bits_sent == 68) { //(sizeof(MM)*8*2 + 4))
		digitalWrite(OUTPUT_PIN, HIGH);
		done = true;

		bits_sent = 0;
		StopInterrupts();
		state = 0;
	} else {
		done = false;
	}
}

/* Main loop.*/
void loop()
{

	if (done == true) {
		delay(950);
		cycles++;
		if (cycles == 1) { //enable CH
			MM1 = 0x00; //parity is 0
			MM2 = 0x00; //0 bit set
			MM3 = 0x03; //3 has 2 bits set
			MM4 = 0x00;
			//      cycles = 0;
		}
		//    if (cycles == 2)
		//    {// send ID 3 to set control to modulating
		//      MM1=0x80;//parity is 1
		//      MM2=0x03;//ID 3
		//      MM3=0x01;
		//      MM4=0x00;
		//      cycles = 0;
		//    }
		//    if (cycles == 3) //worked up to 46 and stopped the burner...
		//    {
		//      MM1=0x10; //write - 1 bit set
		//      MM2=0x39; //ID 57 - 4bits set
		//      MM3=0x25; //37 - 3 bits set
		//      MM4=0x00;
		//      cycles = 0;
		//    }
		if (cycles == 2) { //set water temp to 38 degress
			MM1 = 0x90;
			MM2 = 0x01;
			MM3 = 0x26;
			MM4 = 0x00;
			cycles = 0;
		}
		Serial.println();
		done = false;
		//check radio
		//wait for message from boiler
		//decide on message
		//Serial.print(millis());
		//Serial.print(" - ");
		//Serial.print(originalvalue);
		//Serial.print(" - ");
		StartInterrupts();
	}
}


