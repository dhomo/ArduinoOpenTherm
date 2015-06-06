/*
   OpThLib.cpp
   Copyright 2009 Martijn van den Burg, martijn@[remove-me-first]palebluedot . nl

   This file is part of the OpThLib library for reading OpenTherm (TM)
   communications with Arduino.

   OpThLib is free software: you can redistribute it and/or modify it under
   the terms of the GNU General Public License as published by the Free
   Software Foundation, either version 3 of the License, or (at your option)
   any later version.

   OpThLib is distributed in the hope that it will be useful, but WITHOUT ANY
   WARRANTY; without even the implied warranty of MERCHANTABILITY or
   FITNESS FOR A PARTICULAR PURPOSE.  See the GNU General Public License
   for more details.

   You should have received a copy of the GNU General Public License
   along with OpThLib.  If not, see <http://www.gnu.org/licenses/>.

*/

/* This library is written for the Arduino Duemilanova and Arduino software version 0017.
 *
 * It's purpose is to make data transmitted with the OpenTherm (TM) protocol available
 * for other applications.
 *
 * This library may work with other hardware and/or software. YMMV.
 */


#include "OpThLib.h"


/* Variables used in interrupts */
volatile byte firstEdge = 1;   // used in signal period determination
volatile byte dataReady = 0;   // used in signal period determination
volatile int transitionCount = 0;  // used for error detection
volatile long data1;            // measured signal time [clock ticks]

boolean debugout = 0;

volatile boolean TimerTic = true;

/* Timer2 reload value, globally available */
unsigned int tcnt2;


/* * * * Static functions * * * */


/* Interrupt service routine for signal period measurement. */
static void _ISRsync()
{
	if (firstEdge) {
		TCNT1 = 0;        // reset timer
		firstEdge = 0;
		data1 = 0;
	} else {
		data1 = TCNT1;    // read value of timer
		firstEdge = 1;
		dataReady = 1;
	}
}

/* Interrupt service routine for error detection. */

static void _ISRtransition()
{
	transitionCount++;
}



/* * * * End static functions * * * */


/* Class constructor.
 * _inputPin is tied to the operation of INT0 and Timer1. If you want to use
 * another pin to read the data, you'll have to hack the code.
 */
OpThLib::OpThLib()
{
}


/* Initialise Timer1 and set the pinmode for the pin that we
 * use to listen to the communication.
 */
void OpThLib::init(uint8_t inPin, uint8_t outPin)
{
	_inputPin = inPin;
	// pinMode(_inputPin, INPUT);
	// digitalWrite(_inputPin, HIGH); // enable internal pull-up resistor

	_outputPin = outPin;
	pinMode(_outputPin, OUTPUT);
	digitalWrite(_outputPin, HIGH); // idle state: low voltage
	_frame = 0;

	_average = 465; // debug only
}


// /*
//  * Install the Interrupt Service Routine (ISR) for Timer2 overflow.
//  * This is normally done by writing the address of the ISR in the
//  * interrupt vector table but conveniently done by using ISR()
//  */
ISR(TIMER2_COMPA_vect)
{
	TimerTic = true;
}

// WRITE_TIMER (0) интервал 0,5мс
// READ_TIMER (1) интервал 1мс
void OpThLib::_initTimer(uint8_t mode)
{
	// Configure timer2 in CTC mode
	TCCR2A =  1 << WGM21 | 0 << WGM20;
	TCCR2B &= ~(1 << WGM22); // clear bit, не обязательно и так по дефолту там 0

	/* Select clock source: internal I/O clock */
	ASSR &= ~(1 << AS2); // не обязательно, и так по дефолту там 0

	/* only Output Compare Register A */
	TIMSK2 = (0 << OCIE2A);

	// Для расчёта частоты импульсов, которую можно получить на выводе OCnx, можно использовать формулу
	// Focnx = Fclk/(2*N*(1+OCRnx),
	// где Fclk равен тактовой частоте таймера, а N равен коэффициенту предделителя (например, 8, 32, 64, ...).
	if (mode == WRITE_TIMER) {
		TCCR2B = 0 << CS22 | 1 << CS21 | 1 << CS20; // предделитель/32 один тик 2мкс, 250тиков =0,5мс
		OCR2A = 249; // один тик надо вычитать
	}
	if (mode == READ_TIMER) {
		TCCR2B = 1 << CS22 | 0 << CS21 | 0 << CS20; // предделитель/64 один тик 4мкс, 250тиков =1мс
		OCR2A = 249; // идельно для 1мс /64
	}
}

void OpThLib::getHightLow()
{
	uint16_t val;
	uint32_t tstart;

	digitalWrite(5, HIGH); // enable internal pull-up resistor

	_masterframe = INVALID_DATA << 28 | 0 << 16 | 0;
	_masterframe |= parity_even_bit(_masterframe) << 31;
	Serial.print("_masterframe:"); Serial.println(_masterframe, BIN);
	_sendFrame();

	tstart = millis();
	while ((millis() - tstart) < 1000) {
		val = analogRead(_inputPin);
		_low = min(_low, val);
		_hight = max(_hight, val);
		delayMicroseconds(100);
	}
	_average = (_hight - _low) / 2;

	Serial.print("_low:"); Serial.print(_low);
	Serial.print("_hight:"); Serial.print(_hight);
	Serial.print("_average:"); Serial.println(_average);

}



uint8_t OpThLib::sendMessage(uint8_t msg_type, uint8_t data_id, uint16_t data_value)
{
	_masterframe = msg_type << 28 | data_id << 16 | data_value;
	_masterframe |= parity_even_bit(_masterframe) << 31;
	return sendMessage ();
}

// четность не проверяем!! сами за ней следите
uint8_t OpThLib::sendMessage(uint32_t frame)
{
	_masterframe = frame;
	return sendMessage ();
}


// выполнение этого метода может затянуться на 800мс
// минимум 90мс, а всреднем 170мс
uint8_t OpThLib::sendMessage()
{
	uint8_t result;
	_sendFrame();
	result = waitFrame(); // wait for a master or slave frame
	if ( result == NO_ERROR) {
		result = readFrame();
	}
	return result;
}


void OpThLib::_sendFrame()
{
	bool outHalfBit = 1;
	byte half_bits_sent = 0;
	boolean logic_transition = false;

	TimerTic = false;      // set to 1 to obtain the first sample immediately

	_initTimer(WRITE_TIMER);
	/* enable the timer */
	TIMSK2 |= (1 << OCIE2A);

	// начиная с этого момента 0.5мсек таймер запустился
	// он отслает 34*2 бита и занимает это 34 мсек
	// while (half_bits_sent < FRAME_LENGTH * 2) {
	// 	if (TimerTic) {
	// 		digitalWrite(_outputPin, !outHalfBit);
	// 		half_bits_sent += 1;
	// 		switch (half_bits_sent) {
	// 		// start bit
	// 		// вторая половина, первую мы уже отправили
	// 		case 1:
	// 			outHalfBit = 0;
	// 			break;
	// 		case 2 ... 65:
	// 			if (logic_transition == false) {
	// 				outHalfBit = _masterframe >> 31;
	// 				_masterframe = _masterframe << 1;
	// 				logic_transition = true;
	// 			} else {
	// 				outHalfBit = !outHalfBit;
	// 				logic_transition = false;
	// 			}
	// 			break;
	// 		// end bit
	// 		case 66:
	// 			outHalfBit = 1;
	// 			break;
	// 		case 67:
	// 			outHalfBit = 0;
	// 			break;
	// 		}
	// 		TimerTic = false;
	// 	}
	// }


	// начиная с этого момента 0.5мсек таймер запустился
	// он отслает 34*2 бита и занимает это 34 мсек
	while (half_bits_sent < FRAME_LENGTH * 2) {
		if (TimerTic) {
			
			switch (half_bits_sent) {
			// start bit
			case 0:
				outHalfBit = 1;
				break;			
			case 1:
				outHalfBit = 0;
				break;
			// data bits	
			case 2 ... 65:
				if (logic_transition == false) {
					outHalfBit = _masterframe >> 31;
					_masterframe = _masterframe << 1;
					logic_transition = true;
				} else {
					outHalfBit = !outHalfBit;
					logic_transition = false;
				}
				break;
			// end bit
			case 66:
				outHalfBit = 1;
				break;
			case 67:
				outHalfBit = 0;
				break;
			}

			digitalWrite(_outputPin, !outHalfBit);
			half_bits_sent += 1;
			TimerTic = false;
		}
	}

	// disable the timer
	TIMSK2 &= ~(1 << OCIE2A); // Clear bit
}


/* Measure the period of the bits in the data stream so that we can synchronize
 * with the protocol.
 * - uses _ISRsync
 */
void OpThLib::measureOtPeriod()
{
	int8_t cnt1 = 0;              // loop counter
	int8_t cnt2 = 0;              // loop counter
	uint16_t sum = 0;             // sum of T2 values
	uint16_t average = 0;         // average of T2 values
	uint16_t T2;                  // measured minimum time period

	/* enable Timer1 overflow interrupt so we can detect when it overflows
	 * while we're still waiting for the next rising edge. For example: if we
	 * triggered on the last rising edge of a frame.
	*/
	TIMSK1 = 1 << TOIE1;

	/* attach ISRsync() to INT1, digital pin 3. Pin is kept high by internal
	* pull up. Triggering on the RISING edge doesn't quite work.
	*/
	attachInterrupt(1, _ISRsync, FALLING);

	/* take the average of SAMPLE_SIZE samples of T2, each being the minimum T2 of
	* SAMPLE_SIZE measurements. */
	while (cnt1 < SAMPLE_SIZE) {
		cnt2 = 0;
		T2 = 0xFFFF;

		while (cnt2 < SAMPLE_SIZE) {
			if (dataReady) {          // interrupt handler has performed measurement
				if ( data1 != 0 ) {     // Timer1 overflow occurred
					if ( data1 < T2 ) {    // find smallest shortest time between two rising edges
						T2 = data1;
					}
					cnt2++;                // increment loopcounter after valid data received
				}
				dataReady = 0;          // prepare for next sample
			}
		}
		sum += T2;                  // summarize all samples, for average calculation
		cnt1++;
	}

	detachInterrupt(1);   // don't need the interrupt anymore
	TIMSK1 = 0 << TOIE1;  // disable Timer1 overflow interrupt

	_otPeriod = sum * TICK_TO_USEC / SAMPLE_SIZE;          // calculate average [us]


	/* We want Timer1 to create an overflow interrupt every _otPeriod us, which we
	 * then use as a trigger to go and sample the input signal.
	 * One clock tick = 64/16000000 * 1E6 = TICK_TO_USEC us. Need to time `_otPeriod [us] / TICK_TO_USEC` ticks.
	 * Subtract from timer overflow value (take 0xFFFF, the maximum timer value).
	 * See http://www.uchobby.com/index.php/2007/11/24/arduino-interrupts/
	 */
	_preload = 0xFFFF - (_otPeriod / TICK_TO_USEC);
} // measureOtPeriod

void OpThLib::setPeriod(int32_t per)
{
	_otPeriod = per;
	_preload = 0xFFFF - (_otPeriod / TICK_TO_USEC);
}


/* Wait for the next frame. The amount of time to wait depends on whether a Master
 * or a Slave frame is expected.
 */
int8_t OpThLib::waitFrame()
{
	int8_t waitState = 1;
	uint32_t timeLow;
	unsigned long tstart;
	uint8_t noframe = 0;

	digitalWrite(2, LOW);
	tstart = millis();

	while (waitState) {
		// while (digitalRead(_inputPin) == HIGH) {
		while (analogRead(_inputPin) > _average) { // ждем низкого уровня
			if (millis() - tstart > 500) {
				noframe = 1;
				break;
			}
		}   // do nothing

		timeLow = millis();
		// while (digitalRead(_inputPin) == LOW) {
		while (analogRead(_inputPin) < _average) { // ждем высокого уровня
			if ( millis() - timeLow >= MIN_WAIT_FOR_SLAVE ) {
				waitState = 0;
			}
			if (millis() - tstart > 500) {
				noframe = 1;
				break;
			}
		}

		if (millis() - tstart > 500) {
			noframe = 1;
			break;
		}
	}
	// fall through when signal goes HIGH and waitState == 0
	if (noframe == 1) {
		_errorMsg = "no frame";
		return 1;
	}

	digitalWrite(2, HIGH);
	return 0;
} // waitFrame


/* Read one frame from the wire.
 * - we enter on a rising signal
 * - configure Timer1 to trigger an internal interrupt at every _otPeriod us
 * - do not start the timer immediately (i.e. at the rising edge), but wait for a
 *   period of _otPeriod / TICK_TO_USEC us so that we do not risk sampling right at an edge.
 * - start timer
 * - sample the signal when the timer overflows: this is the correct bitvalue
 * - bad data check: with every HI/LO and LO/HI transition, transitionCount is incremented. When a
 *   new sample is taken we check that the current transitionCount is larger than the transitionCount
 *   of the previous sample. This way, spurious signals can be detected and sampling aborted.
 * - sample FRAME_LENGTH bits
 * - check start- and end bits on the fly
 * - perform parity check on the fly
 *
 * An unsigned long (32 bits) is used to store the 32 data bits. The start and stop bits (both 1)
 * are discarded because they hold no value after the check on them has been done.
 *
 * The functions returns 0 for success and non-zero for error, setting _errorMsg (which
 * can be accessed with the errmsg() function).
 *
 */
int8_t OpThLib::readFrame()
{
	uint8_t cnt = 0;                     // loop counter
	uint8_t parityOdd = 0;              // parity check
	uint8_t previousTransitionCount = 0;
	uint32_t tempFrame = 0;
	uint8_t sample;
	uint8_t result = 0;

	Serial.print("readFrame>");

	TimerTic = true;      // set to 1 to obtain the first sample immediately
	transitionCount = 0;    // incremented inside ISR

	// Wait a quarter of a period (1ms) to get 'into' the period and thus prevent samples on a
	// rising/falling edge.
	// Note: delayMicroseconds() disables interrupts.
	// delayMicroseconds(250);
	delayMicroseconds(50);

	/* Attach _ISRtransition() to INT0 (digital pin 3), to measure signal transitions. */
	// detachInterrupt(1);
	// attachInterrupt(1, _ISRtransition, CHANGE);

	// debugout != debugout;
	// digitalWrite(2, !digitalRead(2));

	_initTimer(READ_TIMER);
	/* enable the timer */
	TIMSK2 |= (1 << OCIE2A);

	/* Go and sample FRAME_LENGTH bits.
	* Note: The first sample is not obtained through timer overflow (because dataReady was
	* initialized to 1), but the remainder of samples is.
	*/
	while ( cnt < FRAME_LENGTH ) {
		if (TimerTic) {
			// TCNT1 = _preload;    // preload timer

			/* Bad data check: number of transitions must have increaseds since previous sample
			* (except when we're measuring the first sample).
			* 'transitionCount' is incremented by INT0 handler 'ISR_transition'.
			*/
			// if ( (cnt > 0) && (transitionCount == previousTransitionCount) ) {
			//  _errorMsg = "bad data";
			//  _frame = 0;
			//  return 1;
			// } else {
			//  previousTransitionCount = transitionCount;
			// }

			// sample = digitalRead(_inputPin);    // get the value of the signal on _inputPin
			sample = analogRead(_inputPin) > _average;

			// debugout != debugout;
			// digitalWrite(2, debugout); // толька для  теста УБРАТЬ ПОТОМ
			digitalWrite(2, !digitalRead(2));
			// Serial.print(sample);// толька для  теста УБРАТЬ ПОТОМ

			/* parity check, toggle parityOdd each time the signal is 1. */
			if ( sample == 1 ) {
				parityOdd = !parityOdd;
			}

			/* First sample is start bit, last is stop bit. They ought to be 1 and we don't store them. */
			if ( (cnt == 0) ) {
				if (sample != 1) {
					_errorMsg = "bad start bit";
					//           Serial.println("0<(bad start bit)readFrame");
					_frame = 0;
					result = 2;
					// break;
				}
			} else if ( (cnt == (FRAME_LENGTH - 1)) ) {
				if (sample != 1) {
					_errorMsg = "bad end bit";
					// Serial.print(tempFrame, BIN);
					// Serial.print("0<(bad end bit)readFrame");
					_frame = 0;
					result = 12;
					// break;
				}
			} else {
				/* store this sample by shifting all bits in the unsigned long to the left,
				 * then store relevant number of bits from the sample value.
				 * Note that the shifting is also done (unnecessarilly) for the first data bit, but since
				 * the initialization value of 'bits' is 0, that doesn't matter.
				 */
				tempFrame <<= 1;
				tempFrame |= sample;
			}
			cnt++;
			TimerTic = false;
		}  // TimerTic
	}  // cnt < FRAME_LENGTH

	// We're done with the interrupt.
	// detachInterrupt(1);

	// disable the timer
	TIMSK2 &= ~(1 << OCIE2A); // Clear bit

	if (result != 0) {
		Serial.print(tempFrame, BIN);
		Serial.println("<(ERROR)readFrame");
		return result;
	}


	/* check parity, should be even. If it isn't then something is
	 * wrong and return with an error.
	*/
	if (parityOdd == 1) {
		_errorMsg = "parity error";
		Serial.print(tempFrame, BIN);
		Serial.println("<(parity error)readFrame");
		_frame = 0;
		return 3;
	} else {
		_frame = tempFrame;               // store measured frame in class' private variable.
	}

	Serial.print(tempFrame, BIN);
	Serial.println("<(no err)readFrame");
	return 0;   // success
} // readFrame


/* Accessors for internal data structures. */


/* Set the frame value manually. Use this for example when receiving the frame from another
 * source (e.g. radio transmitter) to be able to access its content. */
void OpThLib::setFrame(uint32_t frame)
{
	_frame = frame;
}

uint32_t OpThLib::getFrame()
{
	return _frame;
}

char *OpThLib::errmsg()
{
	return _errorMsg;
}

int32_t OpThLib::getPeriod()
{
	return _otPeriod;
}

/* Return message type (a three-bit number) as a byte. */
uint8_t OpThLib::getMsgType()
{
	uint32_t tempFrame = _frame;
	tempFrame >>= 28;                  // shift 28 bits into oblivion
	uint8_t msgType = tempFrame & 0b111;  // take only three bits (masking with B111) 7
	return msgType;
}

/* Return data ID as a single byte. */
uint8_t OpThLib::getDataId()
{
	uint32_t tempFrame = _frame;
	tempFrame >>= 16;
	return (uint8_t)tempFrame;         // type casting discards 2 bytes
}

/* Return data value as an unsigned int. */
uint16_t OpThLib::getDataValue()
{
	return (uint16_t)_frame;           // type casting discards 6 bytes
}

/* Return parity as a single byte. */
uint8_t OpThLib::getParity()
{
	uint32_t tempFrame = _frame;
	tempFrame >>= 31;                 // shift 31 bits into oblivion
	return tempFrame;
}

/* Return 1 if it's a master frame, 0 if it's a slave frame. */
uint8_t OpThLib::isMaster()
{
	if (getMsgType() <= 3) {
		return 1;
	}

	return 0;
}
