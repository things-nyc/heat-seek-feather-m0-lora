/*******************************************************************************
 * Copyright (c) 2015 Thomas Telkamp and Matthijs Kooijman
 *
 * Permission is hereby granted, free of charge, to anyone
 * obtaining a copy of this document and accompanying files,
 * to do whatever they want with them without any restriction,
 * including, but not limited to, copying, modification and redistribution.
 * NO WARRANTY OF ANY KIND IS PROVIDED.
 *
 * This example sends a valid LoRaWAN packet with payload "Hello,
 * world!", using frequency and encryption settings matching those of
 * the The Things Network.
 *
 * This uses ABP (Activation-by-personalisation), where a DevAddr and
 * Session keys are preconfigured (unlike OTAA, where a DevEUI and
 * application key is configured, while the DevAddr and session keys are
 * assigned/generated in the over-the-air-activation procedure).
 *
 * Note: LoRaWAN per sub-band duty-cycle limitation is enforced (1% in
 * g1, 0.1% in g2), but not the TTN fair usage policy (which is probably
 * violated by this sketch when left running for longer)!
 *
 * To use this sketch, first register your application and device with
 * the things network, to set or generate a DevAddr, NwkSKey and
 * AppSKey. Each device should have their own unique values for these
 * fields.
 *
 * Do not forget to define the radio type correctly in config.h.
 *
 *******************************************************************************/

#include <lmic.h>
#include <hal/hal.h>
#include <SPI.h>
#include <OneWire.h>

// TTN Keys
static const PROGMEM u1_t NWKSKEY[16] = { <add key here> };
static const u1_t PROGMEM APPSKEY[16] = { <add key here> };
static const u4_t DEVADDR = <add address here>; // <-- Change this address for every node!

// These callbacks are only used in over-the-air activation, so they are
// left empty here (we cannot leave them out completely unless
// DISABLE_JOIN is set in config.h, otherwise the linker will complain).
void os_getArtEui (u1_t* buf) { }
void os_getDevEui (u1_t* buf) { }
void os_getDevKey (u1_t* buf) { }

// OneWire Setup
OneWire ds(A0);  //the oneWire data pin  -- change as needed
#define DS_POWER_PIN A1  //for turning on and off --DS18B20 power turn off not yet used
    //DS18B20 power can be +3.3, but future low power requirements may make turnoff desireable
#define RBUFFER_LEN 8  //number of stored readings
#define PAYLOAD_LEN   (RBUFFER_LEN + 2)
#define INIT_BYTE 250;  //first byte of payload

//ADDITIONS TO GLOBALS

byte addr[8]; //sensor address on onewire bus
byte readings[RBUFFER_LEN]; // storage for readings
byte head = 0;  //next place to store a reading (better code possible)
byte payload[PAYLOAD_LEN]; // this will eventually be renamed to mydata[], the actual payload

//FUNCTIONS FOR OPERATION

void set_resolution (void) {    //for 18B20 call during setup/initialization
  digitalWrite(DS_POWER_PIN, HIGH); //turn on power to DS18B20
  delay(1000);  //wake up time  probably can be much shorter
  ds.search(addr); // address on 1wire bus
  ds.reset();
  ds.select(addr);
  ds.write(0x4E);
  ds.write (0xFF);  //TempH alarm don't care about value as alarms not read
  ds.write (0xF0);  //TempL alarm
  ds.write (0x3F);  //configuration  -- 10 bit resolution  lsb first apparently handled by library
  ds.reset();
  Serial.println("resolution set");  //for debugging
  //  digitalWrite(DS_POWER_PIN,LOW); //turn off power to DS18B20  not yet implemented
}

byte readTemp(void) {      //for DS18B20
  byte i;
  byte data[12];
  byte present = 0;
  byte output, celsius4;
  
  if (!ds.reset()) return 243;  //code for not present
  ds.select(addr);
  ds.write(0x44, 1);        // start conversion, with parasite power on at the end -- should avoid current drain through 4.7k resistor
  delay(200);     // for 10 bit resolution  change this if resolution changes

  present = ds.reset();  //not sure why this is needed, BUT it is
  ds.select(addr);
  ds.write(0xBE);         // Read Scratchpad

  //keep for future debugging
 /* Serial.print("  Data = ");
  Serial.print(present, HEX);
  Serial.print(" ");*/
  
  for ( i = 0; i < 9; i++) {           // we need 9 bytes
    data[i] = ds.read();
  
  //  Serial.print(data[i], HEX);  //uncomment to debug
  //  Serial.print(" ");   
  
  }  
  
 /* Serial.print(" CRC=");
  Serial.print(OneWire::crc8(data, 8), HEX);
  Serial.println();
   */

  if (OneWire::crc8(addr, 7) != addr[7]) {
    Serial.println("CRC is not valid!");
    return 244;
  }  //code for bad CRC

  // Convert the data to actual temperature
  // because the result is a 16 bit signed integer, it should
  // be stored to an "int16_t" type, which is always 16 bits
  // even when compiled on a 32 bit processor.
  
  int16_t raw = (data[1] << 8) | data[0];
  //left in in case resolution changes -- BUT change to delay must be made manually
  byte cfg = (data[4] & 0x60);
  // at lower res, the low bits are undefined, so let's zero them
  if (cfg == 0x00) raw = raw & ~7;  // 9 bit resolution, 93.75 ms
  else if (cfg == 0x20) raw = raw & ~3; // 10 bit res, 187.5 ms
  else if (cfg == 0x40) raw = raw & ~1; // 11 bit res, 375 ms
  ////  sensor default is 12 bit resolution, 750 ms conversion time
  
  //Serial.println(raw);
  raw = raw + 160;  //add 10 degrees celsius to shift -10 to +50 range to 0 to 60
  if (raw < 0) return 241;  //code for below -10 deg celsius
  if (raw > 960) return 242; //code for above 50 deg celsius (960 = 16*60 --max temperature)
  celsius4 = raw / 4; // in units of 1/4 degree
  return celsius4;  //error codes returned earlier
}


void dataToBuffer (byte newdata) {   // for data from any sensor or other source
  byte index;
  readings[head] = newdata;
  
  // data is now in readings buffer in order received
  // insert data in payload current byte first
  index = head;
  for (byte i = 0 ; i < RBUFFER_LEN  ; i++) {  // do once for each entry in readings
    payload[i + 1] = readings[index];  // start insert with second byte
    if (index == 0) index = RBUFFER_LEN ;
    index--;
  }
    
  head++;  // set up for next data byte
  if (head > (RBUFFER_LEN - 1)) head = 0;
    // insert battery condition byte here -- payload[PAYLOAD_LEN 1 1] = batt cond  last byte
}

//FUNCTIONS USEFUL FOR DEBUGGING

float decodeTempC(byte c4) { // input is celsius
  float output;
  float input = c4;
  if (c4 <= 240) output =  (float(input / 4) - 10); //temperature in range
  if (c4 == 241) {
    Serial.println("below range");
    output =  255;
  }
  if (c4 == 242) {
    Serial.println("above range");
    output =  255;
  }
  if (c4 == 243) {
    Serial.println("no sensor");
    output =  255;
  }
  if (c4 == 244) {
    Serial.println("bad CRC");
    output =  255;
  }
  if (c4 >= 245) {
    Serial.println("unknown code");
    output = 255;
  }
  return output;
}

float decodeTempF(byte c4) { //input is farenheight
  float output;
  float input = c4;
  if (c4 <= 240) output =  (((input / 4) - 10) * 1.8 + 32); //temperature in range
  if (c4 == 241) {
    Serial.println("below range");
    output =  255;
  }
  if (c4 == 242) {
    Serial.println("above range");
    output =  255;
  }
  if (c4 == 243) {
    Serial.println("no sensor");
    output =  255;
  }
  if (c4 == 244) {
    Serial.println("bad CRC");
    output =  255;
  }
  if (c4 >= 245) {
    Serial.println("unknown code");
    output = 255;
  }
  return output;
}


static uint8_t mydata[] = "Hello, world!";
static osjob_t sendjob;

// Schedule TX every this many seconds (might become longer due to duty
// cycle limitations).
const unsigned TX_INTERVAL = 10;

// Pin mapping
const lmic_pinmap lmic_pins = {
    .nss = 8,
    .rxtx = LMIC_UNUSED_PIN,
    .rst = 4,
    .dio = {3, 6, LMIC_UNUSED_PIN},
};

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
            break;
        case EV_RFU1:
            Serial.println(F("EV_RFU1"));
            break;
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
              Serial.println(F("Received "));
              Serial.println(LMIC.dataLen);
              Serial.println(F(" bytes of payload"));
            }
            // Schedule next transmission
            os_setTimedCallback(&sendjob, os_getTime()+sec2osticks(TX_INTERVAL), do_send);
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
         default:
            Serial.println(F("Unknown event"));
            break;
    }
}

void do_send(osjob_t* j){
    // Check if there is not a current TX/RX job running
    if (LMIC.opmode & OP_TXRXPEND) {
        Serial.println(F("OP_TXRXPEND, not sending"));
    } else {
        // Prepare upstream data transmission at the next possible time.
        LMIC_setTxData2(1, mydata, sizeof(mydata)-1, 0);
        Serial.println(F("Packet queued"));
    }
    // Next TX is scheduled after TX_COMPLETE event.
}

void setup() {
    Serial.begin(115200);
    Serial.println(F("Starting"));

    pinMode(DS_POWER_PIN, OUTPUT);
    digitalWrite(DS_POWER_PIN, HIGH); // turn on power to DS18B20
    
    //for 18B20
    ds.search(addr);
    set_resolution();
  
    // initialize readings and payload buffer
    for (byte i = 0 ; i < RBUFFER_LEN ; i++) readings[i] = 0xFF;
    payload[0] = INIT_BYTE;  // initialize constant first byte of payload
    payload[PAYLOAD_LEN-1] = 0xFF;  // no data on battery condition yet

    #ifdef VCC_ENABLE
    // For Pinoccio Scout boards
    pinMode(VCC_ENABLE, OUTPUT);
    digitalWrite(VCC_ENABLE, HIGH);
    delay(1000);
    #endif

    // LMIC init
    os_init();
    // Reset the MAC state. Session and pending data transfers will be discarded.
    LMIC_reset();

    // Set static session parameters. Instead of dynamically establishing a session
    // by joining the network, precomputed session parameters are be provided.
    #ifdef PROGMEM
    // On AVR, these values are stored in flash and only copied to RAM
    // once. Copy them to a temporary buffer here, LMIC_setSession will
    // copy them into a buffer of its own again.
    uint8_t appskey[sizeof(APPSKEY)];
    uint8_t nwkskey[sizeof(NWKSKEY)];
    memcpy_P(appskey, APPSKEY, sizeof(APPSKEY));
    memcpy_P(nwkskey, NWKSKEY, sizeof(NWKSKEY));
    LMIC_setSession (0x1, DEVADDR, nwkskey, appskey);
    #else
    // If not running an AVR with PROGMEM, just use the arrays directly
    LMIC_setSession (0x1, DEVADDR, NWKSKEY, APPSKEY);
    #endif

    #if defined(CFG_eu868)
    // Set up the channels used by the Things Network, which corresponds
    // to the defaults of most gateways. Without this, only three base
    // channels from the LoRaWAN specification are used, which certainly
    // works, so it is good for debugging, but can overload those
    // frequencies, so be sure to configure the full frequency range of
    // your network here (unless your network autoconfigures them).
    // Setting up channels should happen after LMIC_setSession, as that
    // configures the minimal channel set.
    LMIC_setupChannel(0, 868100000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
    LMIC_setupChannel(1, 868300000, DR_RANGE_MAP(DR_SF12, DR_SF7B), BAND_CENTI);      // g-band
    LMIC_setupChannel(2, 868500000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
    LMIC_setupChannel(3, 867100000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
    LMIC_setupChannel(4, 867300000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
    LMIC_setupChannel(5, 867500000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
    LMIC_setupChannel(6, 867700000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
    LMIC_setupChannel(7, 867900000, DR_RANGE_MAP(DR_SF12, DR_SF7),  BAND_CENTI);      // g-band
    LMIC_setupChannel(8, 868800000, DR_RANGE_MAP(DR_FSK,  DR_FSK),  BAND_MILLI);      // g2-band
    // TTN defines an additional channel at 869.525Mhz using SF9 for class B
    // devices' ping slots. LMIC does not have an easy way to define set this
    // frequency and support for class B is spotty and untested, so this
    // frequency is not configured here.
    #elif defined(CFG_us915)
    // NA-US channels 0-71 are configured automatically
    // but only one group of 8 should (a subband) should be active
    // TTN recommends the second sub band, 1 in a zero based count.
    // https://github.com/TheThingsNetwork/gateway-conf/blob/master/US-global_conf.json
    LMIC_selectSubBand(1);
    #endif

    // Disable link check validation
    LMIC_setLinkCheckMode(0);

    // TTN uses SF9 for its RX2 window.
    LMIC.dn2Dr = DR_SF9;

    // Set data rate and transmit power for uplink (note: txpow seems to be ignored by the library)
    LMIC_setDrTxpow(DR_SF7,14);

    // Start job
    do_send(&sendjob);
}

void loop() {
    byte currentTemp;
  
    currentTemp = readTemp();
    Serial.println(readTemp());
    Serial.println(decodeTempC(currentTemp));   //for debugging- returns degrees  celsius as a float
    Serial.println(decodeTempF(currentTemp));  //for debugging- returns degrees  farenheight as a float
    
    //put recent readings into a buffer with most recent data first
    dataToBuffer (currentTemp); //adds data to buffer and payload
 
    //print payload for debugging
    Serial.println(currentTemp);
    Serial.print("Payload    ");
    for (byte i = 0 ; i < PAYLOAD_LEN  ; i++){
      Serial.print(payload[i], DEC);
      Serial.print("  ");
    }
    Serial.println();
    //end of payload printout

    delay (1000);
    os_runloop_once();
}