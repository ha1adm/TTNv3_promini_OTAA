#include "promini_otaa.h"



#ifdef DEBUG
#define DEBUGPRINT(x) Serial.print(x)
#define DEBUGPRINTARG(x,y) Serial.print(x,y)
#define DEBUGPRINTLN(x) Serial.println(x)
#define DEBUGPRINTLNARG(x,y) Serial.println(x, y)
#else
#define DEBUGPRINT(x)
#define DEBUGPRINTARG(x,y)
#define DEBUGPRINTLN(x)
#define DEBUGPRINTLNARG(x,y)
#endif

#define ONE_WIRE_BUS 7
OneWire oneWire(ONE_WIRE_BUS);
DS18B20 sensor(&oneWire);
SPIFlash flash(8 , 0xEF40);

const unsigned BATT_LVL_MIN = 340;
//const unsigned BATT_LVL_OK = 900;

// Schedule TX every this many seconds (might become longer due to duty
// cycle limitations).
const unsigned TX_INTERVAL = 900;
bool transmitting = false;

void do_sendmac(osjob_t* j) {
  // This function is called if we need to process a MAC message
  // Check if there is not a current TX/RX job running
  if (LMIC.opmode & OP_TXRXPEND) {
    DEBUGPRINTLN(F("OP_TXRXPEND, not sending"));
  }
  else {
    // Prepare upstream data transmission at the next possible time.
    byte mydata[1];
    LMIC_setTxData2(0, mydata, sizeof(mydata), 0);
    DEBUGPRINTLN(F("Packet queued (MAC command)"));
  }
  // Next TX is scheduled after TX_COMPLETE event.
}

void printFlashUID(){
uint8_t* MAC = flash.readUniqueId(); 
for (uint8_t i=0;i<8;i++) { 
    DEBUGPRINTARG(MAC[i], HEX); 
    DEBUGPRINT(' '); }
    DEBUGPRINTLN();

}

uint16_t analogOversample(const byte analogPin) {
  uint16_t reading = 0;
  int i = 0;
  while (i < 8)
  {
    i++;
    reading = reading + analogRead(analogPin);
  }
  return reading >> 3;
}

// Reads Analog input and calculates result as Volts according to Voltage divider Resistors
// R1 is the high side of the divider Resistor values in kOhms
int readBattVoltageInt(float r1, float r2, int analogPin, float vcc) {
  for (int i = 0; i < 16; i++) analogRead(analogPin); // first readings are not accurate
  uint16_t ADCreading = analogOversample(analogPin);

  //Convert analog ADC reading to millivolts
  int _voltage = (((ADCreading * vcc / 1024.0) * (r1 + r2) / r2) * 100);
  return _voltage;
}

void resetLMICDuty(int deepsleep_sec){
unsigned long now = millis();
#if defined(CFG_LMIC_EU_like)
  DEBUGPRINTLN(F("Reset CFG_LMIC_EU_like band avail"));
  for (int i = 0; i < MAX_BANDS; i++){
    ostime_t correctedAvail = LMIC.bands[i].avail - ((now / 1000.0 + deepsleep_sec) * OSTICKS_PER_SEC);
    if (correctedAvail < 0)
    {
      correctedAvail = 0;
    }
    LMIC.bands[i].avail = correctedAvail;
    }
  LMIC.globalDutyAvail = LMIC.globalDutyAvail - ((now / 1000.0 + deepsleep_sec) * OSTICKS_PER_SEC);
  if (LMIC.globalDutyAvail < 0)
  {
    LMIC.globalDutyAvail = 0;
  }
  
#else
  Serial.println(F("No DutyCycle recalculation function!"));
#endif
}

void sleep_long() {
DEBUGPRINTLN(F("Sleep long"));
Serial.flush(); // give the serial print chance to complete
delay(20);
for (int i=0; i<int(TX_INTERVAL/8); i++) {
    LowPower.powerDown(SLEEP_8S, ADC_OFF, BOD_OFF);
    Serial.flush(); // give the serial print chance to complete
    }
  DEBUGPRINTLN(F("Sleep long finished"));
}

void check_battery(int16_t batt_minimum) {
    while (readBattVoltageInt(100.0, 100.0, A3, 3.3) <= batt_minimum){
        LowPower.powerDown(SLEEP_8S, ADC_OFF, BOD_OFF);
        resetLMICDuty(8);
        Serial.flush(); // give the serial print chance to complete   
    }

}

// Set LoRaWAN keys defined in lorawan-keys.h.
    static const u1_t PROGMEM DEVEUI[8]  = { OTAA_DEVEUI } ;
    static const u1_t PROGMEM APPEUI[8]  = { OTAA_APPEUI };
    static const u1_t PROGMEM APPKEY[16] = { OTAA_APPKEY };
    // Below callbacks are used by LMIC for reading above values.
    void os_getDevEui (u1_t* buf) { memcpy_P(buf, DEVEUI, 8); }
    void os_getArtEui (u1_t* buf) { memcpy_P(buf, APPEUI, 8); }
    void os_getDevKey (u1_t* buf) { memcpy_P(buf, APPKEY, 16); } 


//static uint8_t mydata[] = "Hello, world!";
static osjob_t sendjob;

// Pin mapping
const lmic_pinmap lmic_pins = {
    .nss = 10,                       
    .rxtx = LMIC_UNUSED_PIN,
    .rst = 9,                       
    .dio = {2, 5, LMIC_UNUSED_PIN}, 
};

void printHex2(unsigned v) {
    v &= 0xff;
    if (v < 16)
        Serial.print('0');
    DEBUGPRINTARG(v, HEX);
}

void onEvent (ev_t ev) {
    DEBUGPRINT(os_getTime());
    DEBUGPRINT(": ");
    switch(ev) {
        case EV_SCAN_TIMEOUT:
            DEBUGPRINTLN(F("EV_SCAN_TIMEOUT"));
            break;
        case EV_BEACON_FOUND:
            DEBUGPRINTLN(F("EV_BEACON_FOUND"));
            break;
        case EV_BEACON_MISSED:
            DEBUGPRINTLN(F("EV_BEACON_MISSED"));
            break;
        case EV_BEACON_TRACKED:
            DEBUGPRINTLN(F("EV_BEACON_TRACKED"));
            break;
        case EV_JOINING:
            DEBUGPRINTLN(F("EV_JOINING"));
            break;
        case EV_JOINED:
            DEBUGPRINTLN(F("EV_JOINED"));
            {
              u4_t netid = 0;
              devaddr_t devaddr = 0;
              u1_t nwkKey[16];
              u1_t artKey[16];
              LMIC_getSessionKeys(&netid, &devaddr, nwkKey, artKey);
              DEBUGPRINT("netid: ");
              DEBUGPRINTLNARG(netid, DEC);
              DEBUGPRINT("devaddr: ");
              DEBUGPRINTLNARG(devaddr, HEX);
              DEBUGPRINT("AppSKey: ");
              for (size_t i=0; i<sizeof(artKey); ++i) {
                if (i != 0)
                  DEBUGPRINT("-");
                printHex2(artKey[i]);
              }
              DEBUGPRINTLN("");
              DEBUGPRINT("NwkSKey: ");
              for (size_t i=0; i<sizeof(nwkKey); ++i) {
                      if (i != 0)
                              Serial.print("-");
                      printHex2(nwkKey[i]);
              }
              DEBUGPRINTLN();
            }
        // Disable link check validation (automatically enabled
        // during join, but because slow data rates change max TX
	    // size, we don't use it in this example.
            LMIC_setLinkCheckMode(0);
            break;
        case EV_JOIN_FAILED:
            DEBUGPRINTLN(F("EV_JOIN_FAILED"));
            break;
        case EV_REJOIN_FAILED:
            DEBUGPRINTLN(F("EV_REJOIN_FAILED"));
            break;
        case EV_TXCOMPLETE:
            DEBUGPRINTLN(F("EV_TXCOMPLETE (includes waiting for RX windows)"));
            if (LMIC.txrxFlags & TXRX_ACK)
              DEBUGPRINTLN(F("Received ack"));
            if (LMIC.dataLen) {
              DEBUGPRINT(F("Received "));
              DEBUGPRINT(LMIC.dataLen);
              DEBUGPRINTLN(F(" bytes of payload"));
            }
            // ready for next transmission
            transmitting = false;
            // Schedule next transmission
            //os_setTimedCallback(&sendjob, os_getTime()+sec2osticks(TX_INTERVAL), do_send);
            break;
        case EV_LOST_TSYNC:
            DEBUGPRINTLN(F("EV_LOST_TSYNC"));
            break;
        case EV_RESET:
            DEBUGPRINTLN(F("EV_RESET"));
            break;
        case EV_RXCOMPLETE:
            // data received in ping slot
            DEBUGPRINTLN(F("EV_RXCOMPLETE"));
            break;
        case EV_LINK_DEAD:
            DEBUGPRINTLN(F("EV_LINK_DEAD"));
            break;
        case EV_LINK_ALIVE:
            DEBUGPRINTLN(F("EV_LINK_ALIVE"));
            break;
        case EV_TXSTART:
            DEBUGPRINTLN(F("EV_TXSTART"));
            break;
        case EV_TXCANCELED:
            DEBUGPRINTLN(F("EV_TXCANCELED"));
            break;
        case EV_RXSTART:
            /* do not print anything -- it wrecks timing */
            break;
        case EV_JOIN_TXCOMPLETE:
            DEBUGPRINTLN(F("EV_JOIN_TXCOMPLETE: no JoinAccept"));
            break;

        default:
            DEBUGPRINT(F("Unknown event: "));
            DEBUGPRINTLN((unsigned) ev);
            break;
    }
}

void do_send(osjob_t* j){
    // Check if there is not a current TX/RX job running
    if (LMIC.opmode & OP_TXRXPEND) {
        Serial.println(F("OP_TXRXPEND, not sending"));
    } else {
        // Prepare upstream data transmission at the next possible time.
        /// Added
          sensor.requestTemperatures();
          while (!sensor.isConversionComplete());  // wait until sensor is ready
          byte payload[4];
          int16_t batt = readBattVoltageInt(100.0, 100.0, A3, 3.3);
          int16_t temp = sensor.getTempC() * 100;;
          payload[0] = batt >> 8;;
          payload[1] = batt & 0xFF;
          payload[2] = temp >> 8;
          payload[3] = temp & 0xFF;  
        // Sends static text every time
        //LMIC_setTxData2(1, mydata, sizeof(mydata)-1, 0);
        // Prepare upstream data transmission at the next possible time.
        LMIC_setTxData2(1, payload, sizeof(payload), 0);
    }
    // Next TX is scheduled after TX_COMPLETE event.
}

void setup() {
    Serial.begin(115200);
    DEBUGPRINTLN(F("Starting"));
    sensor.begin();
    // Check FLASH
    DEBUGPRINT(F("Start flash...")); 
    if (flash.initialize())
      DEBUGPRINTLN(F("Init Flash OK!"));
    else
      DEBUGPRINTLN(F("Init Flash FAIL!"));
    DEBUGPRINT(F("DeviceID: "));
    DEBUGPRINTLNARG(flash.readDeviceId(), HEX);

    // Get Board Temperature
    #ifdef DEBUG 
        sensor.requestTemperatures();
        while (!sensor.isConversionComplete());  // wait until sensor is ready
        DEBUGPRINT(F("Temp: "));
        DEBUGPRINTLN(sensor.getTempC());
    #endif
    // Read Battery voltage
    DEBUGPRINT(F("Batt: "));
    DEBUGPRINTLN(readBattVoltageInt(100.0, 100.0, A3, 3.3));
    // LMIC init
    os_init();
    // Reset the MAC state. Session and pending data transfers will be discarded.
    LMIC_reset();

    // Relax LMIC timing if defined
    #if defined(LMIC_CLOCK_ERROR_PPM)
        uint32_t clockError = 0;
        #if LMIC_CLOCK_ERROR_PPM > 0
            #if LMIC_CLOCK_ERROR_PPM > 4000
                // Allow clock error percentage to be > 0.4%
                # define LMIC_ENABLE_arbitrary_clock_error 1
            #endif    
            clockError = (LMIC_CLOCK_ERROR_PPM / 100) * (MAX_CLOCK_ERROR / 100) / 100;
            LMIC_setClockError(clockError);
        #endif
    #endif
    LMIC.rxDelay = 5;

    // Start job (sending automatically starts OTAA too)
    do_send(&sendjob);
}

void loop() {
    os_runloop_once();
    if (transmitting == false) {
    if (LMIC.pendMacLen > 0) {
      DEBUGPRINTLN(F("Pending MAC message"));
      do_sendmac(&sendjob);
    }
    else {
      Serial.flush(); // give the serial print chance to complete
      delay(20);
      sleep_long();
      resetLMICDuty(TX_INTERVAL);
      // Continue only if the Battery is over a safe limit
      check_battery(BATT_LVL_MIN);
      do_send(&sendjob);
    }

}
}
