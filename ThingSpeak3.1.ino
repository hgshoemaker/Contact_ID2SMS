/*
 * This code is for a Arduino Leonardo board. It has more program space than a UNO
 * and also a real UART pins 0 and 1. Not tested on UNO.
 *
 * The original idea li0r.wordpress.com Alarmino project, has great information on the contact id
 * protocol and hardware to decode DTMF.
 *
 * Contact ID decoding and SMS message buffering and great hardware information
 * Modified to use ESP8266-01 instead of SIM900 GSM board
 * NASCO Alarm Panel SMS Gateway (APSG)
 * For information please visit www.ndtech.com.au
 * Debounce library import as a zip 
 * 
 * ESP8266 WiFi code
 * WiFiEsp Library
 * For more details see: http://yaab-arduino.blogspot.com/p/wifiesp-example-client.html
 * ESP8266 AT Firmware 1.5.4
 * 
 * ThingSpeak.com HTTPThing to Twillio SMS API
 */


// --------------------------------------------------------------
// Libraries
#include <Debounce.h>
#include "WiFiEsp.h"
//----------------------------------------------------------------

#define FIRMWARE_VERSION           6
#define MAX_DTMF_DATA             20
#define CONTACT_ID_LENGTH         16
#define MAX_ALARM_EVENTS          16
#define DTMF_TO_HANDSHAKE_TIME  2000 // Time between last DTMF tone of phone number and sending the handshake tone
#define DTMF_TO_KISSOFF_TIME    1000 // Time between last DTMF tone of contact ID and sending the kissoff tone

#define STATE_INIT_GSM             0
#define STATE_ON_HOOK              1
#define STATE_WAIT_PHONE_NUMBER    2
#define STATE_WAIT_CONTACT_ID      3
#define SMS_CHECK              15000 // Time between SMS checks
#define SMS_SMS_DELAY          15000 // Time before sending a partially empty SMS message
#define SMS_CHECK_TIME_GOOD    10000 // Time between checking ESP module
#define SMS_CHECK_TIME_BAD     20000 // Time between checking ESP module

#define MAX_ZONES                   8
#define MAX_PHONES                  2
#define MAX_ZONE_NAME_LENGTH       15
#define MAX_SMS_MESSAGE           180
#define MAX_SMS_LENGTH            160
#define MAX_SMS_NUMBER             15

const int dtmf8870[]       = { 'D', '1', '2', '3', '4', '5', '6', '7', '8', '9', '0', '*', '#', 'A', 'B', 'C' };
const int dtmfContactID[]  = { 'X', '1', '2', '3', '4', '5', '6', '7', '8', '9', '0', '*', '#', 'D', 'E', 'F' };
const int dtmfValue[]      = {   0,   1,   2,   3,   4,   5,   6,   7,   8,   9,  10,  11,  12,  13,  14,  15 };
const int intValue[]       = {   0,   1,   2,   3,   4,   5,   6,   7,   8,   9,   0,   0,   0,   0,   0,   0 };

//API key for the Thingspeak ThingHTTP already configured
const String apiKey = "your thingspeak thingHTTP apiKey";

// Emulate Serial1 on pins 0/1 if not present
#ifndef HAVE_HWSERIAL1
#include "SoftwareSerial.h"
SoftwareSerial Serial1(0, 1); // RX, TX
#endif

char ssid[] = "your network SSID";            // your network SSID (name)
char pass[] = "your wifi password";        // your network password
int status = WL_IDLE_STATUS;     // the Wifi radio's status
char server[] = "api.thingspeak.com";

// Initialize the Ethernet client object
WiFiEspClient client;



// Pin Mapping (Leonardo)
// http://www.instructables.com/id/Step-by-Step-Guide-to-the-Arduino-Leonardo/step3/More-Analog-Pins/
#define PIN_DTMF_DECODE      2
#define PIN_DTMF_3           4
#define PIN_DTMF_1           6
#define PIN_OFF_HOOK         7
#define PIN_DTMF_0           8
#define PIN_DTMF_2          12
#define PIN_TONE            13

// --------------------------------------------------------------
// Ademco Contact ID Protocol
#define EVENT_QUALIFIER_OPENING                      1
#define EVENT_QUALIFIER_CLOSING                      3
#define EVENT_QUALIFIER_STATUS                       6

#define EVENT_CODE_BURGLAR_BURGLARY                  130
#define EVENT_CODE_BURGLAR_PERIMETER                 131
#define EVENT_CODE_BURGLAR_INTERIOR                  132
#define EVENT_CODE_BURGLAR_24_HOUR_SAFE              133
#define EVENT_CODE_BURGLAR_ENTRY_EXIT                134
#define EVENT_CODE_BURGLAR_DAY_NIGHT                 135
#define EVENT_CODE_BURGLAR_OUTDOOR                   136
#define EVENT_CODE_BURGLAR_TAMPER                    137
#define EVENT_CODE_BURGLAR_NEAR_ALARM                138
#define EVENT_CODE_BURGLAR_INTRUSION_VERIFIER        139
#define EVENT_CODE_GENERAL_ALARM                     140
#define EVENT_CODE_SYSTEM_AC_FAIL                    301
#define EVENT_CODE_OCRA_OPEN_CLOSE                   402
#define EVENT_CODE_OCRA_CANCEL                       406
#define EVENT_CODE_RECENT_CLOSE                      459

#define IN_CANCEL                                    0
#define IN_ALARM                                     1

// --------------------------------------------------------------
// Types
struct AlarmEvent {
  byte type;
  word code;
  word zone;
};

struct ZoneData {
  boolean enabled;
  char    name[MAX_ZONE_NAME_LENGTH + 1];
  boolean alarm;
  boolean cancel;
};

struct PhoneData {
  boolean enabled;
  char    number[MAX_SMS_NUMBER + 1];
};

// --------------------------------------------------------------
// Global variables
Debounce   offHook = Debounce(50, PIN_OFF_HOOK);
int        state;
int        dtmfBit;
int        dtmfDigit;
int        dtmfOffset;
int        dtmfChecksum;
boolean    dtmfDecoded;
boolean    dtmfAvailable;
int        dtmfData[MAX_DTMF_DATA];
AlarmEvent alarmEvents[MAX_ALARM_EVENTS];
byte       alarmEventsHead;
byte       alarmEventsTail;
byte       alarmEventsAvl;
byte       alarmEventsLength;
byte       alarmType;
word       alarmCode;
word       alarmZone;
byte       eventQualifier;
word       eventCode;
word       eventZone;
int        t;
unsigned long smsSendSms;
unsigned long espCheck;
unsigned long smsCheck;
char          smsBuffer[MAX_SMS_MESSAGE];
char          smsNumber[MAX_SMS_NUMBER];
char          smsMessage[MAX_SMS_MESSAGE];
unsigned long now;
unsigned long dtmfTime;


// --------------------------------------------------------------
// Configuration
// true = enabled, Zone description, true = report alarms, true = report restorals
ZoneData zones[MAX_ZONES] = {
  { true, "ZONE ONE",    true, true },
  { true, "ZONE TWO",    true, true },
  { true, "ZONE THREE",  true, true },
  { true, "ZONE FOUR",   true, true },
  { true, "ZONE FIVE",   true, true },
  { true, "ZONE SIX",    true, true },
  { false, "ZONE SEVEN",  false, false },
  { false, "ZONE EIGHT",  false, false }
};

// PhoneData true = use the following number... the phone number is your Twillio registered cell phone #
PhoneData phones[MAX_PHONES] = {
  { true, "+1319xxxxxxx" },
  { false, "+xxxxxxxxxxx" }
};

void handleAlarmEvents(void);
void sendSmsMessage(void);

// --------------------------------------------------------------
// Setup

void setup()
{
  pinMode(PIN_OFF_HOOK, INPUT);
  pinMode(PIN_DTMF_DECODE, INPUT);
  pinMode(PIN_DTMF_0, INPUT);
  pinMode(PIN_DTMF_1, INPUT);
  pinMode(PIN_DTMF_2, INPUT);
  pinMode(PIN_DTMF_3, INPUT);
  pinMode(PIN_TONE, OUTPUT);

  delay(1000);
  Serial.begin(9600);
  delay(1000);
  // initialize serial for ESP module
  Serial1.begin(115200);

  dtmfBit = 0;
  dtmfDigit = 0;
  dtmfDecoded = false;
  dtmfAvailable = false;
  alarmEventsInit(MAX_ALARM_EVENTS);
  state = STATE_ON_HOOK;
  espCheck =0;
  smsSendSms = 0;
  smsCheck = 0;

  Serial.println("Alarm Panel SMS Gateway (APSG)");
  delay(2000);
  WiFi.init(&Serial1);
  // check for the presence of the shield
  if (WiFi.status() == WL_NO_SHIELD) {
    Serial.println("WiFi shield not present");
    // don't continue
    while (true);
  }
  // attempt to connect to WiFi network
  while ( status != WL_CONNECTED) {
    Serial.print("Attempting to connect to WPA SSID: ");
    Serial.println(ssid);
    // Connect to WPA/WPA2 network
    status = WiFi.begin(ssid, pass);
  }
}

void loop()
{
  
  now = millis();
  offHook.update();
  espCheck = now;

  if (state == STATE_WAIT_PHONE_NUMBER || state == STATE_WAIT_CONTACT_ID) {
    if (offHook.read() == 1) {
      // Global reset - hungup detected
      state = STATE_ON_HOOK;
      //Serial.println("STATE_ON_HOOK");
      //Serial.flush();
    } else {
      // Global decode - DTMF if available
      dtmfHandler();
    }
  }

    // Check the ESP8266 buffer every now and then if we arent inside a call from the alarm
  if (state == STATE_ON_HOOK) {
    if (now > smsCheck) {
      smsCheck = now + SMS_CHECK_TIME_GOOD;
    }
  }
  if (status == WL_CONNECTED) {
    // Send SMS messages if the timer has expired
     if ((smsSendSms != 0) && (now > smsSendSms)) {
        sendSmsMessage();
        smsCheck = millis() + SMS_CHECK;
      }
    }

  switch (state) {
    case STATE_ON_HOOK:
      if (offHook.read() == 0) {
        state = STATE_WAIT_PHONE_NUMBER;
        dtmfOffset = 0;
        dtmfChecksum = 0;
        //Serial.println("STATE_WAIT_PHONE_NUMBER");Serial.flush();
      } else {
        handleAlarmEvents();
      }
      break;

    case STATE_WAIT_PHONE_NUMBER:
      if (dtmfAvailable == true) {
        dtmfAvailable = false;
        dtmfTime = now;
        if (dtmfOffset < MAX_DTMF_DATA) {
          dtmfData[dtmfOffset] = dtmfDigit;
          dtmfOffset++;
          //Serial.print((char)(dtmf8870[dtmfDigit]));
        }
      } else {
        if (dtmfOffset > 1) {
          if (now > (dtmfTime + DTMF_TO_HANDSHAKE_TIME)) {
            state = STATE_WAIT_CONTACT_ID;
            dtmfOffset = 0;
            dtmfChecksum = 0;
            //Serial.println("");Serial.flush();
            //Serial.println("STATE_WAIT_CONTACT_ID");Serial.flush();
            tone(PIN_TONE, 1400); delay(100); noTone(PIN_TONE);
            delay(100);
            tone(PIN_TONE, 2300); delay(100); noTone(PIN_TONE);
          }
        }
      }
      break;

    case STATE_WAIT_CONTACT_ID:
      if (dtmfAvailable == true) {
        dtmfAvailable = false;
        dtmfTime = now;
        if (dtmfOffset < MAX_DTMF_DATA) {
          dtmfData[dtmfOffset] = dtmfDigit;
          dtmfOffset++;
          dtmfChecksum = dtmfChecksum + dtmfValue[dtmfDigit];
          //Serial.print((char)dtmfContactID[dtmfDigit]);
          //Serial.print(dtmfDigit);
          //Serial.flush();
        }
      } else {
        if (dtmfOffset > 1) {
          if (now > (dtmfTime + DTMF_TO_KISSOFF_TIME)) {
            if (dtmfOffset == CONTACT_ID_LENGTH) {
              dtmfChecksum = dtmfChecksum - dtmfValue[dtmfDigit];
              for (t = 0; t < dtmfChecksum; t += 15);
              dtmfChecksum = t - dtmfChecksum;
              if (dtmfChecksum == 0) {
                dtmfChecksum = 15;
              }
              if (dtmfChecksum == dtmfValue[dtmfDigit]) {
                alarmType = intValue[dtmfData[6]];
                alarmCode = (((intValue[dtmfData[7]] * 10) + intValue[dtmfData[8]]) * 10) + intValue[dtmfData[9]];
                alarmZone = (((intValue[dtmfData[12]] * 10) + intValue[dtmfData[13]]) * 10) + intValue[dtmfData[14]];
                alarmEventsPut(alarmType, alarmCode, alarmZone);
                //Serial.println(" = OK");
                //Serial.flush();
                tone(PIN_TONE, 1400); delay(800); noTone(PIN_TONE);
              } else {
                //Serial.println(" = CHECKSUM ERROR");
                //Serial.flush();
              }
            } else {
              //Serial.println(" = LENGTH ERROR");
              //Serial.flush();
            }
            dtmfOffset = 0;
            dtmfChecksum = 0;
          }
        }
      }
      break;
  }
}


void sendSMS(String number, String message)
{
  // attempt to connect to WiFi network
  while ( status != WL_CONNECTED) {
    //Serial.print("Attempting to connect to WPA SSID: ");
    Serial.println(ssid);
    // Connect to WPA/WPA2 network
    status = WiFi.begin(ssid, pass);
  }
  // Make a TCP connection to remote host
  if (client.connect(server, 80)){
    Serial.println("Connected to ThingSpeak!");
  

    //should look like this...
    //api.thingspeak.com/apps/thinghttp/send_request?api_key={api key}&number={send to number}&message={text body}

    client.print("GET /apps/thinghttp/send_request?api_key=");
    client.print(apiKey);
    //Serial.println(apiKey);
    client.print("&number=");
    client.print(number);
    client.print("&message=");
    client.print(message);
    client.println(" HTTP/1.1");
    client.print("Host: ");
    client.println(server);
    client.println("Connection: close");
    //Serial.println("Connection: closed");
    client.println();
  }
  else
  {
    Serial.println(F("Connection failed"));
  }

  // Check for a response from the server, and route it
  // out the serial port.
  while (client.connected())
  {
    if ( client.available() )
    {
      char c = client.read();
      Serial.print(c);
    }
  }
  //Serial.println();
  client.stop();
}

String URLEncode(const char* msg)
{
  const char *hex = "0123456789abcdef";
  String encodedMsg = "";

  while (*msg != '\0') {
    if ( ('a' <= *msg && *msg <= 'z')
         || ('A' <= *msg && *msg <= 'Z')
         || ('0' <= *msg && *msg <= '9') ) {
      encodedMsg += *msg;
    }
    else {
      encodedMsg += '%';
      encodedMsg += hex[*msg >> 4];
      encodedMsg += hex[*msg & 15];
    }
    msg++;
  }
  return encodedMsg;
}


void handleAlarmEvents(void) {
  word tempZone;

  while (alarmEventsEmpty() == false) {
    alarmEventsGet(eventQualifier, eventCode, eventZone);
    smsBuffer[0] = 0;
    // NOTE: Only add messages to smsBuffer if you want to send it via SMS
    switch (eventQualifier) {
      case EVENT_QUALIFIER_OPENING:
        switch (eventCode) {
          case EVENT_CODE_BURGLAR_BURGLARY:
            if (eventZone <= MAX_ZONES) {
              tempZone = eventZone - 1;
              if (zones[tempZone].enabled == true && zones[tempZone].alarm == true) {
                sprintf(smsBuffer, "ALARM %s ", zones[tempZone].name);
                //Serial.print(smsBuffer);
              }
            }
            sprintf(smsBuffer, "ALARM BURGLARY ZONE: %d ", eventZone);
            break;

          case EVENT_CODE_BURGLAR_TAMPER:
            sprintf(smsBuffer, "TAMPER %d ", eventZone);
            //Serial.print(smsBuffer);
            break;

          case EVENT_CODE_SYSTEM_AC_FAIL:
            sprintf(smsBuffer, "AC FAIL %d ", eventZone);
            //Serial.print(smsBuffer);
            break;

          case EVENT_CODE_OCRA_OPEN_CLOSE:
            sprintf(smsBuffer, "DISARM SYSTEM USER: %d ", eventZone);
            //Serial.print(smsBuffer);
            break;

          case EVENT_CODE_RECENT_CLOSE:
            sprintf(smsBuffer, "RECENT CLOSE: %d ", eventZone);
            //Serial.print(smsBuffer);
            break;

          case EVENT_CODE_OCRA_CANCEL:
            sprintf(smsBuffer, "CANCEL USER: %d ", eventZone);
            //Serial.print(smsBuffer);
            break;

          case EVENT_CODE_GENERAL_ALARM:
            sprintf(smsBuffer, "General Alarm: %d ", eventZone);
            //Serial.print(smsBuffer);
            break;

          default:
            sprintf(smsBuffer, "ALARM UNKNOWN ET: %d EZ: %d ", eventCode, eventZone);
            //Serial.print(smsBuffer);
            break;
        }
        break;

      case EVENT_QUALIFIER_CLOSING:
        switch (eventCode) {
          case EVENT_CODE_BURGLAR_BURGLARY:
            if (eventZone <= MAX_ZONES) {
              tempZone = eventZone - 1;
              if (zones[tempZone].enabled == true && zones[tempZone].cancel == true) {
                sprintf(smsBuffer, " RESTORAL %s", zones[tempZone].name);
                // Serial.print(smsBuffer);
              }
            }
            sprintf(smsBuffer, "RESTORAL ZONE: %d ", eventZone);
            // Serial.print(smsBuffer);
            break;

          case EVENT_CODE_BURGLAR_TAMPER:
            sprintf(smsBuffer, "RESTORAL TAMPER %d ", eventZone);
            //Serial.print(smsBuffer);
            break;

          case EVENT_CODE_SYSTEM_AC_FAIL:
            sprintf(smsBuffer, "RESTORAL AC FAIL %d ", eventZone);
            //Serial.print(smsBuffer);
            break;

          case EVENT_CODE_OCRA_OPEN_CLOSE:
            sprintf(smsBuffer, "ARM SYSTEM USER: %d ", eventZone);
            //Serial.print(smsBuffer);
            break;

          case EVENT_CODE_RECENT_CLOSE:
            sprintf(smsBuffer, "RECENT CLOSE: %d ", eventZone);
            // Serial.print(smsBuffer);
            break;

          default:
            sprintf(smsBuffer, "CANCEL UNKNOWN ET: %d ZONE: %d ", eventCode, eventZone);
            //Serial.print(smsBuffer);
            break;
        }
        break;

      default:
        sprintf(smsBuffer, "UNKNOWN EQ: %d ET: %d ZONE: %d ", eventQualifier, eventCode, eventZone);
        //Serial.print(smsBuffer);
        break;
    }

    // CHECK If we have a new message to append to existing message
    if (strlen(smsBuffer) > 0) {
      int nml = strlen(smsBuffer);
      int eml = strlen(smsMessage);
      // CHECK If existing message does not have space for new message
      if ((eml + 2 + nml) < MAX_SMS_LENGTH) {
      //if ((eml + nml) > 0) {
        strcat(smsMessage, smsBuffer);
        eml = 0;
      }
      // Add the new message to the existing message
      //strcat(smsMessage, smsBuffer);
      //smsSendSms = millis() + SMS_SMS_DELAY;
      smsSendSms = millis() + SMS_SMS_DELAY;
    }
  }
}

void sendSmsMessage(void) {
  byte p;
  if (strlen(smsMessage) > 0) {
    for (p = 0; p < MAX_PHONES; p++) {
      if (phones[p].enabled == true) {
       //Serial.println("------------------------------");
       Serial.print("SMS TO: "); Serial.println(phones[p].number);
       Serial.println(smsMessage);
       //Serial.println("------------------------------");
       sendSMS(phones[p].number, URLEncode(smsMessage));
       
      }
    }
  }
  smsMessage[0] = 0;
  smsSendSms = 0;

}

// ---------------------------------------------------------------------------
// Read and decode a DTMF tone if available, dont read the same tone twice
void dtmfHandler() {
  int dtmfState = digitalRead(PIN_DTMF_DECODE);
  if (dtmfState == LOW) {
    if (dtmfDecoded == false) {
      dtmfDecoded = true;
      dtmfDigit = 0;
      bitWrite(dtmfDigit, 0, digitalRead(PIN_DTMF_0));
      bitWrite(dtmfDigit, 1, digitalRead(PIN_DTMF_1));
      bitWrite(dtmfDigit, 2, digitalRead(PIN_DTMF_2));
      bitWrite(dtmfDigit, 3, digitalRead(PIN_DTMF_3));
      dtmfAvailable = true;
    }
  } else {
    if (dtmfDecoded == true) {
      dtmfDecoded = false;
    }
  }
}

// ---------------------------------------------------------------------------
// Alarm event buffer
void alarmEventsInit(byte length) {
  alarmEventsHead = 0;
  alarmEventsTail = 0;
  alarmEventsAvl = 0;
  alarmEventsLength = length;
}

boolean alarmEventsEmpty() {
  if (alarmEventsAvl == 0) {
    return true;
  }
  return false;
}

boolean alarmEventsFull() {
  if (alarmEventsAvl == alarmEventsLength) {
    return true;
  }
  return false;
}

boolean alarmEventsPut(byte a, word b, word c) {
  if (alarmEventsAvl == alarmEventsLength) {
    return false;
  }
  alarmEvents[alarmEventsHead].type = a;
  alarmEvents[alarmEventsHead].code = b;
  alarmEvents[alarmEventsHead].zone = c;
  alarmEventsAvl = alarmEventsAvl + 1;
  alarmEventsHead = (alarmEventsHead + 1) & (alarmEventsLength - 1);
  return true;
}

boolean alarmEventsGet(byte &a, word &b, word &c) {
  if (alarmEventsAvl == 0) {
    return false;
  }
  a = alarmEvents[alarmEventsTail].type;
  b = alarmEvents[alarmEventsTail].code;
  c = alarmEvents[alarmEventsTail].zone;
  alarmEventsTail = (alarmEventsTail + 1) & (alarmEventsLength - 1);
  alarmEventsAvl = alarmEventsAvl - 1;
  return true;
}
