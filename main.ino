/*
Agouti+ CREST ENGINEERING PROJECT
Author:
Dylan Kainth <dylan.kainth@dylankainth.com>
*/

#include <Wire.h>
#include "Adafruit_SGP30.h"

Adafruit_SGP30 sgp;

#include <lmic.h>
#include <hal/hal.h>
#include <CayenneLPP.h>
#include "DHT.h"

/* These are constants and variable setting mishmash, don't change these */
#define uS_TO_S_FACTOR 1000000  /* Conversion factor for micro seconds to seconds */
RTC_DATA_ATTR int bootCount = 0; /* Count how many times the device is booted*/
#define DHTTYPE DHT11  // We're using a DHT 11

/* These can be changed */
#define TIME_TO_SLEEP  900        /* Time between transmissions */
#define DHTPIN  15     // DHT11 is connected to GPIO 15
const lmic_pinmap lmic_pins = {
    .nss = 5,
    .rxtx = LMIC_UNUSED_PIN,
    .rst = 12,
    .dio = {26, 2, 4},
}; // define the pinmapping for LORA
#define LEDPIN 16 // define the pin for the LED
#define SGPPOWERPIN 33 // define the pin for the SGP30 power

// for the lora device:
// VCC to 3.3V
// GND to GND
// NSS to GPIO5
// MOSI to GPIO23
// MISO to GPIO19
// SCK to GPIO18
// DIO0 to GPIO21
// DIO1 to GPIO2
// DIO2 to GPIO4

/* Post-Variable assignment initialisation */
DHT dht(DHTPIN, DHTTYPE); /* Initialise DHT */

static uint8_t btn_activated[1] = { 0x01};
static osjob_t sendjob;
// define the LORAWAN appeui, deveui and appkey
// make sure to use LSB (little endian) format, so least significant byte first for the APPEUI and DEVEUI, but not the APPKEY
static const u1_t PROGMEM APPEUI[8]= { 0x4C, 0x53, 0x60, 0x77, 0x07, 0xF9, 0x81, 0x60};
void os_getArtEui (u1_t* buf) { memcpy_P(buf, APPEUI, 8);}

static const u1_t PROGMEM DEVEUI[8]= { 0xF7, 0x01, 0x1B, 0x0B, 0xE9, 0xF9, 0x81, 0x60 };
void os_getDevEui (u1_t* buf) { memcpy_P(buf, DEVEUI, 8);}

static const u1_t PROGMEM APPKEY[16]= {0xFF, 0xF6, 0xB3, 0xB1, 0x60, 0x0A, 0x13, 0x31, 0x23, 0xBD, 0x9F, 0x65, 0x07, 0xBA, 0x21, 0x4F};
void os_getDevKey (u1_t* buf) {  memcpy_P(buf, APPKEY, 16);}

void printHex2(unsigned v) {
    v &= 0xff;
    if (v < 16)
        Serial.print('0');
    Serial.print(v, HEX);
}

int fPort = 1;               // Port to use in transmission


void onEvent (ev_t ev) {
    Serial.print(os_getTime());
    Serial.print(": ");
    switch(ev) {
        case EV_SCAN_TIMEOUT:
            Serial.println(F("EV_SCAN_TIMEOUT"));
            break;
        case EV_BEACON_FOUND:
            Serial.println(F("EV_BEACON_FOUND"));
            break;
        case EV_BEACON_MISSED:
            Serial.println(F("EV_BEACON_MISSED"));
            break;
        case EV_BEACON_TRACKED:
            Serial.println(F("EV_BEACON_TRACKED"));
            break;
        case EV_JOINING:
            Serial.println(F("EV_JOINING"));
            break;
            
        case EV_JOINED:
            Serial.println(F("EV_JOINED"));
            {
              u4_t netid = 0;
              devaddr_t devaddr = 0;
              u1_t nwkKey[16];
              u1_t artKey[16];
              LMIC_getSessionKeys(&netid, &devaddr, nwkKey, artKey);
              Serial.print("netid: ");
              Serial.println(netid, DEC);
              Serial.print("devaddr: ");
              Serial.println(devaddr, HEX);
              Serial.print("AppSKey: ");
              for (size_t i=0; i<sizeof(artKey); ++i) {
                if (i != 0)
                  Serial.print("-");
                printHex2(artKey[i]);
              }
              Serial.println("");
              Serial.print("NwkSKey: ");
              for (size_t i=0; i<sizeof(nwkKey); ++i) {
                if (i != 0)
                  Serial.print("-");
                printHex2(nwkKey[i]);
              }
              Serial.println();
            }
            // Disable link check validation (automatically enabled
            // during join, but because slow data rates change max TX
        // size, we don't use it in this example.
            LMIC_setLinkCheckMode(0);
            break;
        /*
        || This event is defined but not used in the code. No
        || point in wasting codespace on it.
        ||
        || case EV_RFU1:
        ||     Serial.println(F("EV_RFU1"));
        ||     break;
        */
        case EV_JOIN_FAILED:
            Serial.println(F("EV_JOIN_FAILED"));
            break;
        case EV_REJOIN_FAILED:
            Serial.println(F("EV_REJOIN_FAILED"));
            break;
        case EV_TXCOMPLETE:
            Serial.println(F("EV_TXCOMPLETE (includes waiting for RX windows)"));
            if (LMIC.txrxFlags & TXRX_ACK)
              Serial.println(F("Received ack"));

            if (LMIC.dataLen) {
              Serial.print(F("Received "));
              Serial.print(LMIC.dataLen);
              Serial.println(F(" bytes of payload"));

              //------ Added ----------------s
              fPort = LMIC.frame[LMIC.dataBeg - 1];
              Serial.print(F("fPort "));
              Serial.println(fPort);
            
             Serial.println();
             //-----------------------------
            }                    
            Serial.println("Done");
            break;
        case EV_LOST_TSYNC:
            Serial.println(F("EV_LOST_TSYNC"));
            break;
        case EV_RESET:
            Serial.println(F("EV_RESET"));
            break;
        case EV_RXCOMPLETE:
            // data received in ping slot
            Serial.println(F("EV_RXCOMPLETE"));
            break;
        case EV_LINK_DEAD:
            Serial.println(F("EV_LINK_DEAD"));
            break;
        case EV_LINK_ALIVE:
            Serial.println(F("EV_LINK_ALIVE"));
            break;
        /*
        || This event is defined but not used in the code. No
        || point in wasting codespace on it.
        ||
        || case EV_SCAN_FOUND:
        ||    Serial.println(F("EV_SCAN_FOUND"));
        ||    break;
        */
        case EV_TXSTART:
            Serial.println(F("EV_TXSTART"));
            break;
        case EV_TXCANCELED:
            Serial.println(F("EV_TXCANCELED"));
            break;
        case EV_RXSTART:
            /* do not print anything -- it wrecks timing */
            break;
        case EV_JOIN_TXCOMPLETE:
            Serial.println(F("EV_JOIN_TXCOMPLETE: no JoinAccept"));

            // try again to connect
            do_send(&sendjob);
            break;
        
        default:
            Serial.print(F("Unknown event: "));
            Serial.println((unsigned) ev);
            break;
    }
}

void do_send(osjob_t* j){
    // Check if there is not a current TX/RX job running
    if (LMIC.opmode & OP_TXRXPEND) {
        Serial.println(F("OP_TXRXPEND, not sending"));
    } else {

        // turn on the led 
        digitalWrite(16, HIGH);

        union Payload {
            byte data[16];
            char text[16];
        };

        Payload payload = {"Hello World"};

        fPort = 1;

        Serial.print(("temperature: "));
        Serial.println(dht.readTemperature());

        // If you have a temperature / humidity sensor, you can set the absolute humidity to enable the humditiy compensation for the air quality signals
        //float temperature = 22.1; // [°C]
        //float humidity = 45.2; // [%RH]
        //sgp.setHumidity(getAbsoluteHumidity(temperature, humidity));
        
        sgp.IAQmeasure();
        // sgp.IAQmeasureRaw();

        if (! sgp.IAQmeasure()) {
          Serial.println("Measurement failed");

          // set the payload to zero for these metrics
          sgp.TVOC = 0;
          sgp.eCO2 = 0;
          return;
        }
        Serial.print("TVOC "); Serial.print(sgp.TVOC); Serial.print(" ppb\t");
        Serial.print("eCO2 "); Serial.print(sgp.eCO2); Serial.println(" ppm");

        // if (! sgp.IAQmeasureRaw()) {
        //   Serial.println("Raw Measurement failed");
        //   sgp.rawH2 = 0;
        //   sgp.rawEthanol = 0;

        //   return;
        // }
        // Serial.print("Raw H2 "); Serial.print(sgp.rawH2); Serial.print(" \t");
        // Serial.print("Raw Ethanol "); Serial.print(sgp.rawEthanol); Serial.println("");

        CayenneLPP lpp(51);
        lpp.reset();
        // add the temperature to lpp
        lpp.addTemperature(1, dht.readTemperature());

        // add humidity to lpp
        lpp.addRelativeHumidity(2, dht.readHumidity());

        // add the TVOC to lpp
        lpp.addAnalogInput(3, 0);

        // add the eCO2 to lpp
        lpp.addAnalogInput(4, 0);

        // print the lcc buffer
        Serial.print("LPP buffer: ");
        for (int i = 0; i < lpp.getSize(); i++) {
            Serial.print(lpp.getBuffer()[i], HEX);
            Serial.print(" ");
        }
        
        // send the lcc buffer
        LMIC_setTxData2(fPort, lpp.getBuffer(), lpp.getSize(), 0);
        

        //Prepare upstream data transmission at the next possible time.
        // LMIC_setTxData2(fPort, payload.data, sizeof(payload.data), 0);
        // Serial.print("Packet '");
        // Serial.print(payload.text);
        // Serial.println("'");
        // Serial.println(F("Packet queued"));

        delay(100);
        digitalWrite(16, LOW);
    }
    // Next TX is scheduled after TX_COMPLETE event.
}

uint32_t getAbsoluteHumidity(float temperature, float humidity) {
    // approximation formula from Sensirion SGP30 Driver Integration chapter 3.15
    const float absoluteHumidity = 216.7f * ((humidity / 100.0f) * 6.112f * exp((17.62f * temperature) / (243.12f + temperature)) / (273.15f + temperature)); // [g/m^3]
    const uint32_t absoluteHumidityScaled = static_cast<uint32_t>(1000.0f * absoluteHumidity); // [mg/m^3]
    return absoluteHumidityScaled;
}

void setup(){

  // init the led
  pinMode(LEDPIN, OUTPUT);
  pinMode(SGPPOWERPIN, OUTPUT);
    
  Serial.begin(9600); // initialise serial on baud 9600

  // turn on sgp
  digitalWrite(SGPPOWERPIN, HIGH);

  // wait for sgp to start
  delay(5000);

  sgp.begin();

  delay(1000); //Take some time to open up the Serial Monitor

  //Increment boot number and print it every reboot
  ++bootCount;
  Serial.println("Boot number: " + String(bootCount));

  // init DHT
  dht.begin();
 
  // LMIC init
  os_init();

  // Reset the MAC state. Session and pending data transfers will be discarded.
  LMIC_reset();

  do_send(&sendjob);

  // Use with Arduino Pro Mini ATmega328P 3.3V 8 MHz
  // Let LMIC compensate for +/- 1% clock error
  LMIC_setClockError(MAX_CLOCK_ERROR * 10 / 100);

  LMIC_setDrTxpow(DR_SF7,15); // DR_SF7 = SF7, 15 = 15dBm

  // get time since start of program
  int starttime = (millis());

  // loop over until 30 seconds have passed on top of millis()
  while (millis() < starttime + 30000) {
    os_runloop_once();
  }

  digitalWrite(SGPPOWERPIN, LOW);

  /*
  First we configure the wake up source
  We set our ESP32 to wake up every TIME_TO_SLEEP SECONDS
  */
  esp_sleep_enable_timer_wakeup(TIME_TO_SLEEP * uS_TO_S_FACTOR);
  Serial.println("Setup ESP32 to sleep for every " + String(TIME_TO_SLEEP) +
  " Seconds");

  /*
  Next we decide what all peripherals to shut down/keep on
  By default, ESP32 will automatically power down the peripherals
  not needed by the wakeup source, but if you want to be a poweruser
  this is for you. Read in detail at the API docs
  http://esp-idf.readthedocs.io/en/latest/api-reference/system/deep_sleep.html
  Left the line commented as an example of how to configure peripherals.
  The line below turns off all RTC peripherals in deep sleep.
  */
  //esp_deep_sleep_pd_config(ESP_PD_DOMAIN_RTC_PERIPH, ESP_PD_OPTION_OFF);
  //Serial.println("Configured all RTC Peripherals to be powered down in sleep");

  /*
  Now that we have setup a wake cause and if needed setup the
  peripherals state in deep sleep, we can now start going to
  deep sleep.
  In the case that no wake up sources were provided but deep
  sleep was started, it will sleep forever unless hardware
  reset occurs.
  */


  Serial.println("Sleeping...");
  delay(1000);
  Serial.flush(); 
  esp_deep_sleep_start();
}

void loop(){
  // this is left empty
}
