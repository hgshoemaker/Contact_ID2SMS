# Contact_ID2SMS
This decodes Alarm System Contact ID protocol using a Arduino Leonardo the sends SMS using an ESP8266 via ThingSpeak and Twillio 

This code is for a Arduino Leonardo board. It has more program space than a UNO
 and also a real UART pins 0 and 1. Not tested on UNO.
 
 The original idea li0r.wordpress.com Alarmino project, has great information on the contact id
 protocol and hardware to decode DTMF.
 
 Contact ID decoding and SMS message buffering and great hardware information
 Modified to use ESP8266-01 instead of SIM900 GSM board
 NASCO Alarm Panel SMS Gateway (APSG)
 For information please visit www.ndtech.com.au
 Debounce library import as a zip 
  
 ESP8266 WiFi code
 WiFiEsp Library
 For more details see: http://yaab-arduino.blogspot.com/p/wifiesp-example-client.html
 ESP8266 AT Firmware 1.5.4
  
 ThingSpeak.com HTTPThing to Twillio SMS API
