/*
 * HeaterOpThContr.ino
 *
 * Created: 12/11/2014 7:35:58 PM
 * Author: dhomo
 */


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

#include "Lib/OpThLib.h"

#define OUTPUT_PIN (5)
#define INPUT_ANALOG_PIN (5)
#define TEST_PIN (2)
#define NO_ERROR (0)

#define CHECK_BIT(var,pos) ((var) & (1<<(pos)))

//#define OTPERIOD (9997) //in msec. used a little trick to find mine, see below in loop

/*my stuff amvv */
int cycles = 0;



OpThLib OT;  // create new OpTh class instance

/* Setup phase: configure and enable timer2 overflow interrupt */
void setup()
{

	Serial.begin(115200);
	/* Configure the test pin as output */
	//pinMode(OUTPUT_PIN, OUTPUT);

	pinMode(8, OUTPUT);  // Radio enable pin
	digitalWrite(8, HIGH);

	pinMode(4, OUTPUT);         // Radio sleep pin
	digitalWrite(4, LOW);       // ensure the radio is not sleeping

	delay(1000);                // allow the radio to startup

	//OT = OpTh();  // create new OpTh class instance
	OT.init(INPUT_ANALOG_PIN, OUTPUT_PIN);
	OT.getHightLow();

	pinMode(TEST_PIN, OUTPUT);  //  enable test pin
	digitalWrite(TEST_PIN, LOW);

}


/* Main loop.*/
void loop()
{
	unsigned long tstart;
	tstart = millis();

	uint8_t result = 0;

	cycles++;
	switch (cycles) {
	// case 1:
	// 		// 0b00000000
	// 		//      43210
	// 		// 0: CH enable [ CH is disabled, CH is enabled]
	// 		// 1: DHW enable [ DHW is disabled, DHW is enabled]
	// 		// 2: Cooling enable [ Cooling is disabled, Cooling is
	// 		// enabled]
	// 		// 3: OTC active [OTC not active, OTC is active]
	// 		// 4: CH2 enable [CH2 is disabled, CH2 is enabled]
	// 		uint16_t mode;
	// 		mode |= 1 << 0; //CH enable
	// 		mode |= 1 << 1; //DHW enable
	// 		mode |= 0 << 3; //OTC active
	// 		mode = 0b00000011
	// 	result = OT.sendMessage(READ_DATA, 0, mode<<8);
	// 	break;

	case 1:
		// ID 3 Read Slave configuration
		result = OT.sendMessage(READ_DATA, 3, 0);
		break;	
	// case 2:
	// 	// ID 1 set water temp to 40 degress
	// 	result = OT.sendMessage(WRITE_DATA, 1, byte(40)<<8); // тоже самое что умножить на 256
	// 	break;
	// case 3:
	// 	// ID 17 R - Relative Modulation Level f8.8 0..100 Percent modulation between min and max
	// 	// modulation levels. i.e. 0% = Minimum modulation level 100% = Maximum modulation level
	// 	result = OT.sendMessage(READ_DATA, 17, 0);
	// case 4:
	// 	// ID 25 - CH water temperature
	// 	result = OT.sendMessage(READ_DATA, 25, 0);
	// case 5:
	// 	// ID 26 - DHW temperature
	// 	result = OT.sendMessage(READ_DATA, 26, 0);
	// case 6:
	// 	// ID 28 - Outside temperature
	// 	result = OT.sendMessage(READ_DATA, 27, 0);
	// case 7:
	// 	// ID 28 - return water temperature
	// 	result = OT.sendMessage(READ_DATA, 28, 0);
	default:
		cycles = 0;
		// if nothing else matches, do the default
		// default is optional
	}

	if(result == NO_ERROR){
	    ParseFrame();
	} else {
		Serial.print("ERR:");
		Serial.println(OT.errmsg());
	}

	// цикл длительностью 1 сек, когда все что надо сделали, тупо ждем окончания цикла
	while ((millis() - tstart) <= 1000) {
	}
	Serial.print("-------------------------millis:");
	Serial.println(millis());
	digitalWrite(2, !digitalRead(2));
}

/* Main loop.*/
// void loop()
// {
// 	unsigned long tstart;
// 	tstart = millis();

// 	OT.getHightLow();

// // цикл длительностью 1 сек, когда все что надо сделали, тупо ждем окончания цикла
// 	while ((millis() - tstart) <= 1000) {
// 	}
// }


void ParseFrame()
{
	uint8_t msg_type = OT.getMsgType();
	uint8_t data_id = OT.getDataId();
	uint16_t data_value = OT.getDataValue();
	int t;

	switch (data_id) {
	case 0:  // status

		// bit: description [ clear/0, set/1]
		// 0: fault indication [ no fault, fault ]
		// 1: CH mode [CH not active, CH active]
		// 2: DHW mode [ DHW not active, DHW active]
		// 3: Flame status [ flame off, flame on ]
		// 4: Cooling status [ cooling mode not active, cooling
		// mode active ]
		// 5: CH2 mode [CH2 not active, CH2 active]
		// 6: diagnostic indication [no diagnostics, diagnostic event]
		// 7: reserved
		Serial.print("|ID0-Status: ");
		Serial.print(data_value, BIN);
		if (CHECK_BIT(data_value, 0)){
			Serial.print("fault ");
		}
		if (CHECK_BIT(data_value, 1)){
			Serial.print("CH ");
		}
		if (CHECK_BIT(data_value, 2)){
			Serial.print("DHW ");
		}
		if (CHECK_BIT(data_value, 3)){
			Serial.print("FLAME ");
		}
		Serial.println("|");
		break;	
	case 1:  // Control setpoint
		t = data_value / 256;  // don't care about decimal values
		Serial.print("|Control setpoint: ");
		Serial.print(t);
		Serial.println("C|");		
		break;

	case 3:  // ID 3  Slave configuration
		// bit: description [ clear/0, set/1]
		// 0: DHW present [ dhw not present, dhw is present ]
		// 1: Control type [ modulating, on/off ]
		// 2: Cooling config [ cooling not supported,
		// cooling supported]
		// 3: DHW config [instantaneous or not-specified,
		// storage tank]
		// 4: Master low-off&pump control function [allowed,
		// not allowed]
		// 5: CH2 present [CH2 not present, CH2 present]
		Serial.print("|ID3-SlaveCfg: ");
		Serial.print(data_value, BIN);
		if (CHECK_BIT(data_value, 0)){
			Serial.print("DHW_present ");
		}
		if (CHECK_BIT(data_value, 1)){
			Serial.print("Control type(on/off) ");
		} else { Serial.print("Control_type(modulating) "); }
		
		if (CHECK_BIT(data_value, 2)){
			Serial.print("cooling_supported ");
		}
		if (CHECK_BIT(data_value, 3)){
			Serial.print("DHW_config(storage tank) ");
		}
		if (CHECK_BIT(data_value, 4)){
			Serial.print("Master_low-off&pump_ctrl_func(not allowed) ");
		}
		if (CHECK_BIT(data_value, 5)){
			Serial.print("CH2_present ");
		}
		Serial.println("|");
		break;	
	case 17:  // Relative Modulation Level 
		t = data_value / 256;  // don't care about decimal values
		Serial.print("|Modulation flame: ");
		Serial.print(t);
		Serial.println("%|");		
		break;	
	case 25:  // boiler temp
		t = data_value / 256;
		Serial.print("|boiler temp: ");
		Serial.print(t);
		Serial.print("C|");
		break;
	case 26:  // boiler temp
		t = data_value / 256;
		Serial.print("|DHW temperature: ");
		Serial.print(t);
		Serial.print("C|");
		break;	
	case 27:  // boiler temp
		t = data_value / 256;
		Serial.print("|Outside temperature: ");
		Serial.print(t);
		Serial.print("C|");
		break;
	
	}  // switch
}


