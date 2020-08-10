#include <Arduino.h>
#include <EEPROM.h>
#include <ESP8266WiFi.h>
#include <WiFiManager.h>
#include <ESP8266mDNS.h>
#include <ArduinoOTA.h>

// * Include settings
#include "settingsMainOven.h"

// * Include other modules
#include "OvenDisplay.h"

// * Initiate WIFI client
// WiFiClient espClient;

// * Gets called when WiFiManager enters configuration mode
void configModeCallback(WiFiManager *myWiFiManager)
{
  Serial.println(F("Entered config mode"));
  Serial.println(WiFi.softAPIP());

  // * If you used auto generated SSID, print it
  Serial.println(myWiFiManager->getConfigPortalSSID());
}

// **********************************
// * EEPROM helpers                 *
// **********************************

String read_eeprom(int offset, int len)
{
  Serial.print(F("read_eeprom()"));

  String res = "";
  for (int i = 0; i < len; ++i)
  {
    res += char(EEPROM.read(i + offset));
  }
  return res;
}

void write_eeprom(int offset, int len, String value)
{
  Serial.println(F("write_eeprom()"));
  for (int i = 0; i < len; ++i)
  {
    if ((unsigned)i < value.length())
    {
      EEPROM.write(i + offset, value[i]);
    }
    else
    {
      EEPROM.write(i + offset, 0);
    }
  }
}

// ******************************************
// * Callback for saving WIFI config        *
// ******************************************

bool shouldSaveConfig = false;

// * Callback notifying us of the need to save config
void save_wifi_config_callback ()
{
  Serial.println(F("Should save config"));
  shouldSaveConfig = true;
}

// **********************************
// * Setup OTA                      *
// **********************************

void setup_ota()
{
  Serial.println(F("Arduino OTA activated."));

  // * Port defaults to 8266
  ArduinoOTA.setPort(8266);

  // * Set hostname for OTA
  ArduinoOTA.setHostname(HOSTNAME);
  ArduinoOTA.setPassword(OTA_PASSWORD);

  ArduinoOTA.onStart([]()
  {
    Serial.println(F("Arduino OTA: Start"));
  });

  ArduinoOTA.onEnd([]()
  {
    Serial.println(F("Arduino OTA: End (Running reboot)"));
  });

  ArduinoOTA.onProgress([](unsigned int progress, unsigned int total)
  {
    Serial.printf("Arduino OTA Progress: %u%%\r", (progress / (total / 100)));
  });

  ArduinoOTA.onError([](ota_error_t error)
  {
    Serial.printf("Arduino OTA Error[%u]: ", error);
    if (error == OTA_AUTH_ERROR)
    {
      Serial.println(F("Arduino OTA: Auth Failed"));
    }
    else if (error == OTA_BEGIN_ERROR)
    {
      Serial.println(F("Arduino OTA: Begin Failed"));
    }
    else if (error == OTA_CONNECT_ERROR)
    {
      Serial.println(F("Arduino OTA: Connect Failed"));
    }
    else if (error == OTA_RECEIVE_ERROR)
    {
      Serial.println(F("Arduino OTA: Receive Failed"));
    }
    else if (error == OTA_END_ERROR)
    {
      Serial.println(F("Arduino OTA: End Failed"));
    }
  });

  ArduinoOTA.begin();
  Serial.println(F("Arduino OTA init is finished"));
}

// **********************************
// * Setup MDNS discovery service   *
// **********************************

void setup_mdns()
{
  Serial.println(F("Starting MDNS responder service"));

  bool mdns_result = MDNS.begin(HOSTNAME);
  if (mdns_result)
  {
    MDNS.addService("http", "tcp", 80);
  }
}

// **********************************
// * Setup Main                     *
// **********************************

void setup() {
  // put your setup code here, to run once:

  // * Configure Serial and EEPROM
  Serial.begin(BAUD_RATE);
  EEPROM.begin(512);
    
  // * WiFiManager local initialization. Once its business is done, there is no need to keep it around
  WiFiManager wifiManager;

  // * Reset settings - uncomment for testing
  // wifiManager.resetSettings();

  // * Set callback that gets called when connecting to previous WiFi fails, and enters Access Point mode
  wifiManager.setAPCallback(configModeCallback);

  // * Set timeout
  wifiManager.setConfigPortalTimeout(WIFI_TIMEOUT);

  // * Set save config callback
  wifiManager.setSaveConfigCallback(save_wifi_config_callback);

  // * Fetches SSID and pass and tries to connect
  // * Reset when no connection after 10 seconds
  if (!wifiManager.autoConnect(WIFI_NETWORK, WIFI_PASSWD))
  {
    Serial.println(F("Failed to connect to WIFI and hit timeout"));

   // * Reset and try again, or maybe put it to deep sleep
    ESP.reset();
    delay(WIFI_TIMEOUT);
  }

  // * Save the custom parameters to FS
  if (shouldSaveConfig)
  {
    Serial.println(F("Saving WiFiManager config"));

    write_eeprom(134, 1, "1");        // * 134 --> always "1"
    EEPROM.commit();
  }

  // * If you get here you have connected to the WiFi
  Serial.println(F("Connected to WIFI..."));

  // * Configure OTA
  setup_ota();

  // * Startup MDNS Service
  setup_mdns();

  IPAddress ip;

  ip = WiFi.localIP();
  Serial.print("IP Address = ");
  Serial.println(ip);

  beginDisplay();
}

// **********************************
// * Loop                           *
// **********************************

void loop() {
  // put your main code here, to run repeatedly:
  ArduinoOTA.handle();
  read_softw_serial();
  loopDisplay();


}