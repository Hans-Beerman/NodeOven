// **********************************
// * OvenDisplay Settings           *
// **********************************

#include ".passwd.h"
/* include your own .passwd.h file, with the following information in it:

#pragma once

#define HOSTNAME "YOUR_HOSTNAME"

#define WIFI_NETWORK "YOUR_SSID"
#define WIFI_PASSWD "YOUR_PASSWD"

#define OTA_PASSWD "YOUR_OTA_PASSWD"
*/

// * Baud rate for both hardware and software serial
#define BAUD_RATE 115200
#define BAUD_RATE_SOFTWARE_SERIAL 19200

// * RX pin
#define SOFTW_SERIAL_RX D1
// * TX pin
#define SOFTW_SERIAL_TX D2

// * Max messageReceived length
#define SOFTW_MAXLINELENGTH 128

#ifndef HOSTNAME
// * The hostname of our little creature
#define HOSTNAME "YOUR_HOSTNAME"
#endif

// * WiFi credentials
#ifndef WIFI_NETWORK
#define WIFI_NETWORK "YOUR_SSID"
#define WIFI_PASSWD "YOUR_PASSWD"
#endif

// * The password used for OTA
#ifndef OTA_PASSWORD
#define OTA_PASSWORD "YOUR_OTA_PASSWD"
#endif

// * Wifi timeout in milliseconds
#define WIFI_TIMEOUT 30000

