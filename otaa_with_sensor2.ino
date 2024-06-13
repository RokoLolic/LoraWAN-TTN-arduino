#include <lmic.h>
#include <hal/hal.h>
#include <SPI.h>

#include "Zanshin_BME680.h"  // Include the BME680 Sensor library

//
// For normal use, we require that you edit the sketch to replace FILLMEIN
// with values assigned by the TTN console. However, for regression tests,
// we want to be able to compile these scripts. The regression tests define
// COMPILE_REGRESSION_TEST, and in that case we define FILLMEIN to a non-
// working but innocuous value.
//
#ifdef COMPILE_REGRESSION_TEST
# define FILLMEIN 0
#else
# warning "You must replace the values marked FILLMEIN with real values from the TTN control panel!"
# define FILLMEIN (#dont edit this, edit the lines that use FILLMEIN)
#endif

// This EUI must be in little-endian format, so least-significant-byte
// first. When copying an EUI from ttnctl output, this means to reverse
// the bytes. For TTN issued EUIs the last bytes should be 0xD5, 0xB3,
// 0x70.
static const u1_t PROGMEM APPEUI[8]={ 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00, 0x00 };
void os_getArtEui (u1_t* buf) { memcpy_P(buf, APPEUI, 8);}

// This should also be in little endian format, see above.
static const u1_t PROGMEM DEVEUI[8]={ 0x41, 0x73, 0x06, 0xD0, 0x7E, 0xD5, 0xB3, 0x70 };
void os_getDevEui (u1_t* buf) { memcpy_P(buf, DEVEUI, 8);}

// This key should be in big endian format (or, since it is not really a
// number but a block of memory, endianness does not really apply). In
// practice, a key taken from ttnctl can be copied as-is.
static const u1_t PROGMEM APPKEY[16] = { 0xA1, 0x27, 0x12, 0xF7, 0xBB, 0xCD, 0x06, 0x20, 0x1C, 0xF9, 0x6F, 0x6B, 0xD8, 0x47, 0xBE, 0xA8 };
void os_getDevKey (u1_t* buf) {  memcpy_P(buf, APPKEY, 16);}

//static uint8_t mydata[] = "Hello, world!";
//uint8_t mydata[8];
static osjob_t sendjob;
const uint16_t SERIAL_SPEED{9600};  ///< Set the baud rate for Serial I/O
BME680_Class BME680;  ///< Create an instance of the BME680 class
// int32_t  temp=0, humidity=0, pressure=0, gas=0;  // BME readings
// int32_t * temp, humidity, pressure, gas;  // BME readings
// long* tempP;
// long* humidityP;
// long* pressureP;
// long* gasP;

// Schedule TX every this many seconds (might become longer due to duty
// cycle limitations).
const uint8_t PROGMEM TX_INTERVAL = 60;

// Pin mapping
const lmic_pinmap lmic_pins = { //VIN = 3.3V, GND = GND, EN = ? G0 = 4, SCK = 13, MISO = 12, MOSI = 11, CS = 10, RST = 2, G1 = 5, G2 = 7
    .nss = 6, 
    .rxtx = LMIC_UNUSED_PIN,
    .rst = 5,
    .dio = {2, 3, 4}
}; //max ma 19.15

/*
nss = 10,
    .rxtx = LMIC_UNUSED_PIN,
    .rst = 2,
    .dio = {4, 5, 7}

    .nss = 6, 
    .rxtx = LMIC_UNUSED_PIN,
    .rst = 5,
    .dio = {2, 3, 4}
*/

void onEvent (ev_t ev) {
    
    switch(ev) {

        case EV_JOINED:
            //Serial.println(F("EV_JOINED"));
            
            // Disable link check validation (automatically enabled
            // during join, but because slow data rates change max TX
	    // size, we don't use it in this example.
      
            LMIC_setLinkCheckMode(0);
            //os_setTimedCallback(&sendjob, os_getTime()+sec2osticks(2), do_send);

            break;
        /*
        || This event is defined but not used in the code. No
        || point in wasting codespace on it.
        ||
        || case EV_RFU1:
        ||     Serial.println(F("EV_RFU1"));
        ||     break;
        */
        case EV_TXCOMPLETE:
            /*Serial.println(F("EV_TXCOMPLETE (includes waiting for RX windows)"));
            if (LMIC.txrxFlags & TXRX_ACK)
              Serial.println(F("Received ack"));
            if (LMIC.dataLen) {
              Serial.print(F("Received "));
              Serial.print(LMIC.dataLen);
              Serial.println(F(" bytes of payload"));
            }*/
            // Schedule next transmission
            //BME680.getSensorData(temp, humidity, pressure, gas, false);  // Get readings
            os_setTimedCallback(&sendjob, os_getTime()+sec2osticks(TX_INTERVAL), do_send);
            break;
        
        default:
            Serial.print(F("Unknown event: "));
            Serial.println((unsigned) ev);
            break;
    }
}

void do_send(osjob_t* j){
    // Check if there is not a current TX/RX job running
  //int32_t temp =  2147483640;
  //int32_t  humidity=0, pressure=0, gas=0;
  int32_t  temp, humidity, pressure, gas;  // BME readings
    /*if (LMIC.opmode & OP_TXRXPEND) {
        Serial.println(F("OP_TXRXPEND, not sending"));
    } else {*/
  //BME680.triggerMeasurement();
  BME680.getSensorData(temp, humidity, pressure, gas);  // Get readings
  //BME680.getSensorData(tempP, humidityP, pressureP, gasP);
  //int port = 10;
  delay(1000);
  
  /*temp=0;
  humidity=0;
   pressure=0;
   gas=0;*/
  
  //static uint8_t mydata[8];

  static uint8_t mydata[4];

  // //uint8_t wert=temp/100;
  // mydata[0] = (uint8_t)temp/100;
  // //wert=gas/1000;
  // mydata[1] = (uint8_t)gas/1000;
  // //wert=humidity/1000;
  // mydata[2] = (uint8_t)humidity/1000;
  // //wert=1013 - pressure/100;
  // mydata[3] = (uint8_t)(1013-pressure/100);
  uint8_t wert=temp/100;
  mydata[0] = wert;
  wert=gas/1000;
  mydata[1] = wert;
  wert=humidity/1000;
  mydata[2] = wert;
  wert=(1013 - pressure/100);
  mydata[3] = wert;

  // uint8_t *p = (uint8_t *)&temp;
  // mydata[1] = p[0];
  // mydata[0] = p[1];
  // p = (uint8_t *)&humidity;
  // mydata[3] = p[0];
  // mydata[2] = p[1];
  // p = (uint8_t *)&pressure;
  // mydata[5] = p[0];
  // mydata[4] = p[1];
  // p = (uint8_t *)&gas;
  // mydata[7] = p[0];
  // mydata[6] = p[1];
  
  // int wert=temp/10;
  // //Serial.println(wert);
  // mydata[0] = wert >> 8;
  // mydata[1] = wert & 0xFF;
  // wert=humidity/100;
  // mydata[2] = wert >> 8;
  // mydata[3] = wert & 0xFF;
  // wert=pressure/10;
  // mydata[4] = wert >> 8;
  // mydata[5] = wert & 0xFF;
  // wert=gas/100;
  // mydata[6] = wert >> 8;
  // mydata[7] = wert & 0xFF;

  // int wert=temp/10;
  // Serial.println(wert);
  
  // mydata[0] = temp[3];
  // mydata[1] = temp[2];
  // mydata[2] = humidity[3];
  // mydata[3] = humidity[2];
  // mydata[4] = pressure[3];
  // mydata[5] = pressure[2];
  // mydata[6] = gas[3];
  // mydata[7] = gas[2];

  // mydata[0] = *temp >> 8;
  // mydata[1] = *temp & 0xFF;
  // //wert=humidity/100;
  // mydata[2] = *humidity >> 8;
  // mydata[3] = *humidity & 0xFF;
  // //wert=pressure/10;
  // mydata[4] = *pressure >> 8;
  // mydata[5] = *pressure & 0xFF;
  // //wert=gas/100;
  // mydata[6] = *gas >> 8;
  // mydata[7] = *gas & 0xFF;
  // Check if there is not a current TX/RX job running
  if (LMIC.opmode & OP_TXRXPEND) {
    Serial.println(F("OP_TXRXPEND, not sending"));
    } else {
    // Prepare upstream data transmission at the next possible time.
    LMIC_setTxData2(1, mydata, sizeof(mydata), 0); // Sende
    Serial.println(F("Packet queued"));
    }
// Next TX is scheduled after TX_COMPLETE event.
    
        /*// Prepare upstream data transmission at the next possible time.
        LMIC_setTxData2(1, mydata, sizeof(mydata), 0);
        Serial.println(F("Packet queued"));
    }*/
    // Next TX is scheduled after TX_COMPLETE event.
}

void setup() {
  Serial.begin(SERIAL_SPEED);  // Start serial port at Baud rate
  Serial.println(F("Starting"));
  // gasP=&gas;
  // tempP=&temp;
  // pressureP = &pressure;
  // humidityP = &humidity;
  BME680.begin(I2C_STANDARD_MODE);
  BME680.setOversampling(TemperatureSensor, Oversample1);  // Use enumerated type values
  BME680.setOversampling(HumiditySensor, Oversample1);     // Use enumerated type values
  BME680.setOversampling(PressureSensor, Oversample1);     // Use enumerated type values
  BME680.setIIRFilter(IIR2);  // Use enumerated type values
  BME680.setGas(320, 150);  // 320�c for 150 milliseconds

  
  //BME680.triggerMeasurement();
  // LMIC init
  os_init();
  // Reset the MAC state. Session and pending data transfers will be discarded.

  LMIC_reset();

  /*LMIC_setClockError(MAX_CLOCK_ERROR * 3 / 100);
  LMIC_setLinkCheckMode(0);
  LMIC.dn2Dr = DR_SF9;
  LMIC_setDrTxpow(DR_SF7,14);*/
//LMIC_selectSubBand(1);
  //BME680.getSensorData(temp, humidity, pressure, gas, false);
  // Start job (sending automatically starts OTAA too)
  do_send(&sendjob);
}

void loop() {
    os_runloop_once();
}
