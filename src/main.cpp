// Modified from Github proffalken/HeltecGPS.ino
// @MBConsultingUK"
#include <lmic.h>
#include <hal/hal.h>
#include <SPI.h>
#include <U8x8lib.h>
#include <CayenneLPP.h>
#include <Wire.h>
#include <AHT10.h>

#define  V2

uint8_t readStatus = 0;
AHT10 myAHT10(AHT10_ADDRESS_0X38);

float temp,pa,hum,alt;
int cnt=0;
 
// the OLED used
U8X8_SSD1306_128X64_NONAME_SW_I2C u8x8(/* clock=*/ 15, /* data=*/ 4, /* reset=*/ 16);
 
// Schedule TX every this many seconds (might become longer due to duty
// cycle limitations).
const unsigned TX_INTERVAL = 10;

// Create the LPP object
CayenneLPP lpp(51);


// LoRa Pins
#define LoRa_RST  14  // GPIO 14
#define LoRa_CS   18  // GPIO 18
#define LoRa_DIO0 26  // GPIO 26
#define LoRa_DIO2 32  // GPIO 32

#ifdef V2 //WIFI Kit series V1 not support Vext control
  #define LoRa_DIO1    35   // GPIO35 -- SX127x's IRQ(Interrupt Request) V2
#else
  #define LoRa_DIO1    33   // GPIO33 -- SX127x's IRQ(Interrupt Request) V1
#endif


#define USE_JOINING
 
#ifdef USE_JOINING
  // OTAA join keys
  // This EUI must be in little-endian format, so least-significant-byte
  // first. When copying an EUI from ttnctl output, this means to reverse
  // the bytes. For TTN issued EUIs the last bytes should be 0xD5, 0xB3,
  // 0x70.  
  //(lsb)
  static const u1_t PROGMEM APPEUI[8] = { 0x8B, 0x65, 0x00, 0xF0, 0x7E, 0xD5, 0xB3, 0x70 };
  void os_getArtEui (u1_t* buf) {
    memcpy_P(buf, APPEUI, 8);
  }
 
  // This should also be in little endian format, see above.
  //(lsb)
  static const u1_t PROGMEM DEVEUI[8] = { 0xF1, 0x8F, 0xDA, 0xBB, 0xDD, 0x70, 0x3F, 0x00 };
  void os_getDevEui (u1_t* buf) {
    memcpy_P(buf, DEVEUI, 8);
  }
 
  // This key should be in big endian format (or, since it is not really a
  // number but a block of memory, endianness does not really apply). In
  // practice, a key taken from ttnctl can be copied as-is.
  // The key shown here is the semtech default key.
  // (msb)
  static const u1_t PROGMEM APPKEY[16] = { 0xDA, 0x5A, 0x42, 0xB8, 0x71, 0x50, 0x2E, 0xBA, 0x68, 0x38, 0xC5, 0x28, 0xDE, 0x07, 0xC2, 0x26 };
  void os_getDevKey (u1_t* buf) {
    memcpy_P(buf, APPKEY, 16);
  }
 
#else
  // ABP keys
  //UNO1
  // LoRaWAN NwkSKey, network session key (msb)
  static const PROGMEM u1_t NWKSKEY[] =  { 0xA6, 0xC3, 0x0F, 0xB2, 0x91, 0xDB, 0x55, 0xC5, 0x31, 0x82, 0x53, 0xD4, 0x08, 0x08, 0x7A, 0x00 };
 
  // LoRaWAN AppSKey, application session key (msb)
  static const u1_t PROGMEM APPSKEY[] = { 0x54, 0xBE, 0x2D, 0xE6, 0xB6, 0xB3, 0xF7, 0xC2, 0xD0, 0x33, 0x72, 0xB5, 0x27, 0x20, 0xD6, 0x00 };
 
  // LoRaWAN end-device address (DevAddr)
  static const u4_t DEVADDR = 0x26011500;
 
  void os_getArtEui (u1_t* buf) { }
  void os_getDevEui (u1_t* buf) { }
  void os_getDevKey (u1_t* buf) { }
 
#endif
 
//static uint8_t mydata[] = {13, 37};
static osjob_t sendjob;
 
// Pin mapping
const lmic_pinmap lmic_pins = {
    .nss = LoRa_CS,
    .rxtx = LMIC_UNUSED_PIN,
    .rst = LoRa_RST,
    .dio = { LoRa_DIO0, LoRa_DIO1, LoRa_DIO2 },
};

void do_send(osjob_t* j) {

  // Check if there is not a current TX/RX job running
  if (LMIC.opmode & OP_TXRXPEND) {
    Serial.println(F("OP_TXRXPEND, not sending"));
    u8x8.drawString(0, 7, "OP_TXRXPEND, not sent");
  } else {
    // Read the sensors and pack up the data
      temp = myAHT10.readTemperature();
      //pa   = bme.readPressure() / 100.0F;
      hum  = myAHT10.readHumidity();
      //alt  = bme.readAltitude(SEALEVELPRESSURE_HPA);
  
      Serial.print("Temp:");
      Serial.print(temp);
      Serial.println(" C");
  
      Serial.print("Pa:");
      Serial.print(pa);
      Serial.println(" Pa");
  
      Serial.print("Hum:");
      Serial.print(hum);
      Serial.println(" %");
  
      Serial.print("Alt:");
      Serial.print(alt);
      Serial.println(" m");

      //display sensor
      u8x8.setCursor(0, 3);
      u8x8.printf("T %.2f,H %.2f", temp, hum);

      lpp.reset();
      lpp.addTemperature(2, temp);
      lpp.addRelativeHumidity(3, hum);
     
    // Prepare upstream data transmission at the next possible time.
    LMIC_setTxData2(2, lpp.getBuffer(), lpp.getSize(), 0);
    Serial.println(F("Packet queued"));

    u8x8.drawString(0, 7, "PACKET QUEUED");
    digitalWrite(BUILTIN_LED, HIGH);
  }
  // Next TX is scheduled after TX_COMPLETE event.
}
 
 void onEvent (ev_t ev) {

  Serial.print(os_getTime());
  u8x8.setCursor(0, 2);
  u8x8.printf("TIME %i", os_getTime());
  Serial.print(": ");
  switch (ev) {
    case EV_SCAN_TIMEOUT:
      Serial.println(F("EV_SCAN_TIMEOUT"));
      u8x8.clearLine(7);
      u8x8.drawString(0, 7, "EV_SCAN_TIMEOUT");
      break;
    case EV_BEACON_FOUND:
      Serial.println(F("EV_BEACON_FOUND"));
      u8x8.clearLine(7);
      u8x8.drawString(0, 7, "EV_BEACON_FOUND");
      break;
    case EV_BEACON_MISSED:
      Serial.println(F("EV_BEACON_MISSED"));
      u8x8.clearLine(7);
      u8x8.drawString(0, 7, "EV_BEACON_MISSED");
      break;
    case EV_BEACON_TRACKED:
      Serial.println(F("EV_BEACON_TRACKED"));
      u8x8.clearLine(7);
      u8x8.drawString(0, 7, "EV_BEACON_TRACKED");
      break;
    case EV_JOINING:
      Serial.println(F("EV_JOINING"));
      u8x8.clearLine(7);
      //u8x8.drawString(0, 7, "                ");
      u8x8.drawString(0, 7, "EV_JOINING");
      break;
    case EV_JOINED:
      Serial.println(F("EV_JOINED"));
      u8x8.clearLine(7);
      u8x8.drawString(0, 7, "EV_JOINED ");
      LMIC_setDrTxpow(DR_SF7, 14); //added fixed SF after join for longer range messages
      // Disable link check validation (automatically enabled
      // during join, but not supported by TTN at this time).
      LMIC_setLinkCheckMode(0);
      break;
    case EV_RFU1:
      Serial.println(F("EV_RFU1"));
      u8x8.clearLine(7);
      u8x8.drawString(0, 7, "EV_RFUI");
      break;
    case EV_TXSTART:
      Serial.println(F("EV_TXSTART"));
      u8x8.clearLine(7);
      u8x8.drawString(0, 7, "EV_TXSTART");
      break;      
    case EV_JOIN_FAILED:
      Serial.println(F("EV_JOIN_FAILED"));
      u8x8.clearLine(7);
      u8x8.drawString(0, 7, "EV_JOIN_FAILED");
      break;
    case EV_REJOIN_FAILED:
      Serial.println(F("EV_REJOIN_FAILED"));
      u8x8.clearLine(7);
      u8x8.drawString(0, 7, "EV_REJOIN_FAILED");
      //break;
      break;
    case EV_TXCOMPLETE:
      u8x8.setCursor(0, 6);
      u8x8.printf("PACKET# %i",cnt);
      Serial.print("PACKET#");
      Serial.println(cnt);
      cnt=cnt+1;
      
      Serial.println(F("EV_TXCOMPLETE (includes waiting for RX windows)"));
      u8x8.clearLine(7);
      u8x8.drawString(0, 7, "EV_TXCOMPLETE");
      digitalWrite(BUILTIN_LED, LOW);
      if (LMIC.txrxFlags & TXRX_ACK) {
        Serial.println(F("Received ack"));
        u8x8.clearLine(7);
        u8x8.drawString(0, 7, "Received ACK");
      }
    
      if (LMIC.dataLen) {
        Serial.print(F("Data Received "));
        u8x8.drawString(0, 5, "RX ");
        Serial.print(LMIC.dataLen);
        u8x8.setCursor(4, 5);
        u8x8.printf("%i bytes", LMIC.dataLen);
        Serial.print(F(" bytes of payload 0x"));
        for (int i = 0; i < LMIC.dataLen; i++) {
              if (LMIC.frame[LMIC.dataBeg + i] < 0x10) {
                   Serial.print(F("0"));
              }
               Serial.print(LMIC.frame[LMIC.dataBeg + i], HEX);
              }
               Serial.println();
        u8x8.setCursor(0, 6);
        u8x8.printf("RSSI %d SNR %.1d", LMIC.rssi, LMIC.snr);
      }
   

      // Schedule next transmission
      os_setTimedCallback(&sendjob, os_getTime() + sec2osticks(TX_INTERVAL), do_send);
      break;
    case EV_LOST_TSYNC:
      Serial.println(F("EV_LOST_TSYNC"));
      u8x8.clearLine(7);
      u8x8.drawString(0, 7, "EV_LOST_TSYNC");
      break;
    case EV_RESET:
      Serial.println(F("EV_RESET"));
      u8x8.clearLine(7);
      u8x8.drawString(0, 7, "EV_RESET");
      break;
    case EV_RXCOMPLETE:
      // data received in ping slot
      Serial.println(F("EV_RXCOMPLETE"));
      u8x8.clearLine(7);
      u8x8.drawString(0, 7, "EV_RXCOMPLETE");
      break;
    case EV_LINK_DEAD:
      Serial.println(F("EV_LINK_DEAD"));
      u8x8.clearLine(7);
      u8x8.drawString(0, 7, "EV_LINK_DEAD");
      break;
    case EV_LINK_ALIVE:
      Serial.println(F("EV_LINK_ALIVE"));
      u8x8.clearLine(7);
      u8x8.drawString(0, 7, "EV_LINK_ALIVE");
      break;
    default:
      Serial.println(F("Unknown event"));
      u8x8.clearLine(7);
      u8x8.setCursor(0, 7);
      u8x8.printf("UNKNOWN EVENT %d", ev);
      break;
  }
}
 
 
void setup() {
 
  Serial.begin(115200);
  if (!myAHT10.begin()) {
		Serial.println("Could not find a valid AHT10 sensor, check wiring!");
		while (1);
	}
  u8x8.begin();
  u8x8.setFont(u8x8_font_chroma48medium8_r);
  
  u8x8.drawString(0, 1, "LoRaWAN Thailand");

  // I2c default address is 0x76, if the need to change please modify bme.begin(Addr)

  if (!myAHT10.begin()) {
        Serial.println("No sensor device found, check line or address!");
        while (1);
  }

  SPI.begin(5, 19, 27);
 
  // LMIC init
  os_init();
  // Reset the MAC state. Session and pending data transfers will be discarded.
  LMIC_reset();
 
#ifndef USE_JOINING
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
#endif
 
  LMIC_setDrTxpow(DR_SF8, 14); //set join at SF8 with power 14
 
  pinMode(BUILTIN_LED, OUTPUT);
  digitalWrite(BUILTIN_LED, LOW);
 
  // Start job (sending automatically starts OTAA too)
  do_send(&sendjob);
}
 
void loop() {
  os_runloop_once();
}
