/*
      Copyright 2015-2018 Dirk-Willem van Gulik <dirkx@webweaving.org>
      Copyright 2020      Hans Beerman <hans.beerman@xs4all.nl>
                          Stichting Makerspace Leiden, the Netherlands.

   Licensed under the Apache License, Version 2.0 (the "License");
   you may not use this file except in compliance with the License.
   You may obtain a copy of the License at

       http://www.apache.org/licenses/LICENSE-2.0

   Unless required by applicable law or agreed to in writing, software
   distributed under the License is distributed on an "AS IS" BASIS,
   WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
   See the License for the specific language governing permissions and
   limitations under the License.
*/

#ifndef ESP32
#error "The MetalNode uses an ESP32 based board (Olimex ESP32-PoE)!"
#endif

// An Olimex ESP32-PoE is used (default is POESP board (Aart board))
#define ESP32_PoE


// for debugging
// see DEBUGIT flag in platformio.ini

// i2C params
#define RFID_SDA_PIN    (13)
#define RFID_SCL_PIN    (16)

#include <Arduino.h>
#include <PowerNodeV11.h>
#include <ACNode.h>
#include <RFID.h>   // NFC version
#include <SIG2.h>
#include <Cache.h>
#include <EEPROM.h>

#include ".passwd.h"
/* If using WiFi instead of a fixed ethernet connection, and/or OTA:

Add Passwd.h to the NodeOven/src directoy containing the following information

#pragma once

#define WIFI_NETWORK "YOUR_SSID"
#define WIFI_PASSWD "YOUR_PASSWD"

#define OTA_PASSWD "YOUR_OTA_PASSWD"
*/

#include "pid.h"


#define MACHINE "nodeoven"

#define SOLID_STATE_RELAIS_PIN  (33)
#define SIGNAL_LAMP_PIN         (32)
#define FAN_PIN                 (0)
#define KWH_METER_PIN           (35)

#define SERIAL_RX_PIN               36
#define SERIAL_TX_PIN               4

#define SERIAL_MAXLINELENGTH        128
#define SERIAL2_BAUDRATE            19200

#define DISPLAY_UPDATE_INTERVAL     1000 // in ms


// See https://mailman.makerspaceleiden.nl/mailman/private/deelnemers/2019-February/019837.html

// Introduced by alex - 2020-01-8

// Clear EEProm + Cache button
// Press BUT1 on Olimex ESP32 PoE module before (re)boot of node
// keep BUT1 pressed for at least 5 s
// After the release of BUT1 node will restart with empty EEProm and empty cache
#define CLEAR_EEPROM_AND_CACHE_BUTTON         (34)
#define CLEAR_EEPROM_AND_CACHE_BUTTON_PRESSED (LOW)
#define MAX_WAIT_TIME_BUTTON_PRESSED          (4000)  // in ms


#ifdef WIFI_NETWORK
ACNode node = ACNode(MACHINE, WIFI_NETWORK, WIFI_PASSWD);
#else
ACNode node = ACNode(MACHINE);
#endif

RFID reader = RFID(false); // don't use tags already stored in cache, to see in the server who has used the oven

MqttLogStream mqttlogStream = MqttLogStream();
TelnetSerialStream telnetSerialStream = TelnetSerialStream();

// Use software SPI: CS, DI, DO, CLK
Adafruit_MAX31856 thermo = Adafruit_MAX31856(15, 5, 2, 14);


#ifdef OTA_PASSWD
OTA ota = OTA(OTA_PASSWD);
#endif

// LED aartLed = LED(SYSTEM_LED);    // defaults to the aartLed - otherwise specify a GPIO.

typedef enum {
  BOOTING, OUTOFORDER,      // device not functional.
  REBOOT,                   // forcefull reboot
  TRANSIENTERROR,           // hopefully goes away level error
  NOCONN,                   // sort of fairly hopless (though we can cache RFIDs!)
  WAITINGFORCARD,           // waiting for card.
  CHECKINGCARD,
  CLEARSTATUS,
  APPROVED,
  REJECTED,
  OVENON,
  OVENOFF,
} machinestates_t;


#define NEVER (0)

struct {
  const char * label;                   // name of this state
  LED::led_state_t ledState;            // flashing pattern for the aartLED. Zie ook https://wiki.makerspaceleiden.nl/mediawiki/index.php/Powernode_1.1.
  time_t maxTimeInMilliSeconds;         // how long we can stay in this state before we timeout.
  machinestates_t failStateOnTimeout;   // what state we transition to on timeout.
  unsigned long timeInState;
  unsigned long timeoutTransitions;
  unsigned long autoReportCycle;
} state[OVENOFF + 1] =
{
  { "Booting",                LED::LED_ERROR,                   120 * 1000, REBOOT,         0 },
  { "Out of order",           LED::LED_ERROR,                   120 * 1000, REBOOT,         5 * 60 * 1000 },
  { "Rebooting",              LED::LED_ERROR,                   120 * 1000, REBOOT,         0 },
  { "Transient Error",        LED::LED_ERROR,                     5 * 1000, WAITINGFORCARD, 5 * 60 * 1000 },
  { "No network",             LED::LED_FLASH,                        NEVER, NOCONN,         0 },
  { "Waiting for card",       LED::LED_IDLE,                         NEVER, WAITINGFORCARD, 0 },
  { "Checking card",          LED::LED_PENDING,                   5 * 1000, WAITINGFORCARD, 0 },
  { "Clear status",           LED::LED_PENDING,                      NEVER, WAITINGFORCARD, 0 },
  { "Approved card",          LED::LED_PENDING,                  60 * 1000, CLEARSTATUS,    0 },
  { "Rejected",               LED::LED_ERROR,                     5 * 1000, CLEARSTATUS,    0 },
  { "Device(s) switched on",  LED::LED_ON,  MAX_OVEN_ON_TIME * 3600 * 1000, OVENOFF,        0 },
  { "Switch oven off",        LED::LED_ON,                           NEVER, WAITINGFORCARD, 0 },
};

enum messID {
  e_Beat = 1,
  e_Prev,
  e_Next,
  e_On,
  e_Off
};

enum displayMessageID {
	e_BeatAnswer = 1,
	e_CurrentSchedule,
	e_ScheduleName,
	e_CurrentSegment,
	e_Mode,
	e_Goal,
	e_TimeLeft,
	e_Empty,
	e_IsOn,
	e_IsOff,
	e_Fault,
	e_Solved,
	e_OvenTemp,
	e_Authenticate,
	e_ClearMessage,
	e_Status
};

unsigned long laststatechange = 0, lastReport = 0;
static machinestates_t laststate = OUTOFORDER;
machinestates_t machinestate = BOOTING;

// to handle onconnect only once (only after reboot)
static bool firstOnConnectTime = true;

PIDController * _OvenController;

char reportStr[128];

ESP32WebServer myWebServer(80);

unsigned long wattPulses = 0;
bool kWhPinIsHigh = false;
unsigned long startPinChange = 0;

char messageReceived[SERIAL_MAXLINELENGTH];

bool firstTimeSerialBeatReceived = true;

char tmpStr[255];

unsigned long displayUpdateStartTime = 0;

int prevSchedule = -1;
int prevSegment = -1;
bool clearMessageSent = false;

bool thermoCoupleFault = false;

void updateDisplayTop() {
  String tmpStr;
  char resultStr[SERIAL_MAXLINELENGTH];

  if (prevSchedule != _OvenController->getSelectedSchedule()) {
    prevSchedule = _OvenController->getSelectedSchedule();
    prevSegment = -1;
    sprintf(resultStr, "%d:%d", e_CurrentSchedule, _OvenController->getSelectedSchedule());
    Serial2.println(resultStr);
    tmpStr = _OvenController->getScheduleName();
    sprintf(resultStr, "%d:%s", e_ScheduleName, tmpStr.c_str());
    Serial2.println(resultStr);
    if (_OvenController->getScheduleIsEmpty()) {
      sprintf(resultStr, "%d", e_Empty);
      Serial2.println(resultStr);
    } else {
      sprintf(resultStr, "%d", e_ClearMessage);
      Serial2.println(resultStr);
    }
  }
}

void updateDisplayBottom() {
  String tmpStr;
  char resultStr[SERIAL_MAXLINELENGTH];

  if ((machinestate == OVENON) || _OvenController->ovenIsSwitchedOn()) {
    if (prevSegment != _OvenController->getCurrentSegment()) {
      prevSegment = _OvenController->getCurrentSegment();
      sprintf(resultStr, "%d:%d", e_CurrentSegment, _OvenController->getCurrentSegment());
      Serial2.println(resultStr);
      sprintf(resultStr, "%d:%s", e_Mode, _OvenController->getCurrentMode().c_str());
      Serial2.println(resultStr);
      sprintf(resultStr, "%d:%d", e_Goal, (int)_OvenController->getCurrentGoal());
      Serial2.println(resultStr);
      clearMessageSent = false;
    }
    sprintf(resultStr, "%d:%lu", e_TimeLeft, _OvenController->getTimeLeft());
    Serial2.println(resultStr);
  } else {
    if (!clearMessageSent) {
      sprintf(resultStr, "%d", e_ClearMessage);
      Serial2.println(resultStr);
      clearMessageSent = true;
    }
  }
}

void decodeMessageReceived(int messageID) {
  //String tmpStr;
  char resultStr[SERIAL_MAXLINELENGTH];
 

  switch (messageID) {
  case e_Beat:
    // BEAT received, answer with BEAT
    sprintf(resultStr, "%d", e_BeatAnswer);
    Serial2.println(resultStr);
    if (firstTimeSerialBeatReceived) {
      firstTimeSerialBeatReceived = false;
      updateDisplayTop();
    }
    break;

  case e_Prev:
    // Select previous schedule
    // Check if oven is switched off
    if ((machinestate == OVENON) || _OvenController->ovenIsSwitchedOn()) {
      // skip message
    } else {
      _OvenController->selectSchedule(false);
      updateDisplayTop();
    }
    break;

  case e_Next:
    // Select next schedule
    //Check if oven is switched off
    if ((machinestate == OVENON) || _OvenController->ovenIsSwitchedOn()) {
      // skip message
    } else {
      _OvenController->selectSchedule(true);
      updateDisplayTop();
    }
    break;
      
  case e_On:
    // Switch oven on (if needed/possible)
    // Check if oven is already on
    if ((machinestate == OVENON) || _OvenController->ovenIsSwitchedOn()) {
      // skip request, oven is already on
      return;
    }
 
    // Check if schedule is empty
    if (_OvenController->getScheduleIsEmpty()) {
      sprintf(resultStr, "%d", e_Empty);
      Serial2.println(resultStr);
      return;
    }
 
    // Check if user is approved
    if (machinestate != APPROVED) {
      sprintf(resultStr, "%d", e_Authenticate);
      Serial2.println(resultStr);

      return;
    }

    // Switch oven on
    _OvenController->switchOvenOn();
    sprintf(resultStr, "%d", e_IsOn);
    Serial2.println(resultStr);
    sprintf(resultStr, "%d:0", e_Status); // clear status
    Serial2.println(resultStr);
    machinestate = OVENON;
    prevSegment = -1;
    updateDisplayBottom();
    break;

  case e_Off:
    // switch oven off
    _OvenController->switchOvenOff();
    sprintf(resultStr, "%d", e_IsOff);
    Serial2.println(resultStr);
    machinestate = WAITINGFORCARD;
    sprintf(resultStr, "%d:0", e_Status); // clear status
    Serial2.println(resultStr);
    clearMessageSent = false;
    updateDisplayBottom();
    break;
  }
}

void readSerial2()
{
  if (Serial2.available()) {
    //memset(messageReceived, 0, sizeof(messageReceived));

    while (Serial2.available())
    {
      int len = Serial2.readBytesUntil('\n', messageReceived, SERIAL_MAXLINELENGTH - 1);
      messageReceived[len] = 0;
#ifdef DEBUGIT      
      // Serial.printf("Message received: %s\n\r", messageReceived);
#endif      

      int messageId = atoi(messageReceived);
      decodeMessageReceived(messageId);
    }
  }
}

void displayLoop() {
  if ((millis() - displayUpdateStartTime) > DISPLAY_UPDATE_INTERVAL) {
    displayUpdateStartTime = millis();
    if ((machinestate == OVENON) || _OvenController->ovenIsSwitchedOn()) {
      updateDisplayBottom();
    } else {
      updateDisplayTop();
    }
    double currentOvenTemp = _OvenController->getThermoCoupleTemp();
    if ((currentOvenTemp > -300) && !_OvenController->getTempFault()) {
      sprintf(tmpStr, "%d:%d", e_OvenTemp, (int)currentOvenTemp);
      Serial2.println(tmpStr);
      if (thermoCoupleFault) {
        sprintf(tmpStr, "%d", e_Solved);
        Serial2.println(tmpStr);
        thermoCoupleFault = false;
      }
    } else {
      if (_OvenController->getTempFault()) {
        if (!thermoCoupleFault) {
          thermoCoupleFault = true;
          sprintf(tmpStr, "%d", e_Fault);
          Serial2.println(tmpStr);
        }
      }
    }
  }
}

void countkWhPulses() {
  int currentPinValue = 0;
  currentPinValue = digitalRead(KWH_METER_PIN);
  if (( currentPinValue == HIGH) && !kWhPinIsHigh && (millis() - startPinChange) > 50) {
    startPinChange = millis();
    wattPulses++;
    kWhPinIsHigh = true;
  } else {
    if ((currentPinValue == LOW) && kWhPinIsHigh && (millis() - startPinChange) > 50) {
      startPinChange = millis();
      kWhPinIsHigh = false;
    }
  }
}

void checkClearEEPromAndCacheButtonPressed(void) {
  unsigned long ButtonPressedTime;
  unsigned long currentSecs;
  unsigned long prevSecs;
  bool firstTime = true;

  // check CLEAR_EEPROM_AND_CACHE_BUTTON pressed
  pinMode(CLEAR_EEPROM_AND_CACHE_BUTTON, INPUT);
  // check if button is pressed for at least 3 s
  Log.println("Checking if the button is pressed for clearing EEProm and cache");
  ButtonPressedTime = millis();  
  prevSecs = MAX_WAIT_TIME_BUTTON_PRESSED / 1000;
  Log.print(prevSecs);
  Log.print(" s");
  while (digitalRead(CLEAR_EEPROM_AND_CACHE_BUTTON) == CLEAR_EEPROM_AND_CACHE_BUTTON_PRESSED) {
    if ((millis() - ButtonPressedTime) >= MAX_WAIT_TIME_BUTTON_PRESSED) {
      if (firstTime == true) {
        Log.print("\rPlease release button");
        firstTime = false;
      }
    } else {
      currentSecs = (MAX_WAIT_TIME_BUTTON_PRESSED - millis()) / 1000;
      if ((currentSecs != prevSecs) && (currentSecs >= 0)) {
        Log.print("\r");
        Log.print(currentSecs);
        Log.print(" s");
        prevSecs = currentSecs;
      }
    }
  }
  if (millis() - ButtonPressedTime >= MAX_WAIT_TIME_BUTTON_PRESSED) {
    Log.print("\rButton for clearing EEProm and cache was pressed for more than ");
    Log.print(MAX_WAIT_TIME_BUTTON_PRESSED / 1000);
    Log.println(" s, EEProm and Cache will be cleared!");
    // Clear EEPROM
    EEPROM.begin(1024);
    wipe_eeprom();
    Log.println("EEProm cleared!");
    // Clear cache
    prepareCache(true);
    Log.println("Cache cleared!");
    // wait until button is released, than reboot
    while (digitalRead(CLEAR_EEPROM_AND_CACHE_BUTTON) == CLEAR_EEPROM_AND_CACHE_BUTTON_PRESSED) {
      // do nothing here
    }
    Log.println("Node will be restarted");
    // restart node
    ESP.restart();
  } else {
    Log.println("\rButton was not (or not long enough) pressed to clear EEProm and cache");
  }
}


int inputIsHigh = 0;

void handleNotFound() {
  myWebServer.send(404, "text/plain", "404: Not found"); // Send HTTP status 404 (Not Found) when there's no handler for the URI in the request
}


void setup() {
  Serial.begin(115200);
  Serial.println("\n\n\n");
  Serial.println("Booted: " __FILE__ " " __DATE__ " " __TIME__ );

  Serial2.begin(SERIAL2_BAUDRATE, SERIAL_8N1, SERIAL_RX_PIN, SERIAL_TX_PIN);
  Serial2.setRxBufferSize(256);
  Serial2.flush();

  sprintf(tmpStr, "%d:1", e_Status); // Please wait, controller is starting
  Serial2.println(tmpStr);

  pinMode(KWH_METER_PIN, INPUT);

  checkClearEEPromAndCacheButtonPressed();

  _OvenController = new PIDController();
  _OvenController->begin(SOLID_STATE_RELAIS_PIN, FAN_PIN, SIGNAL_LAMP_PIN);

  node.set_mqtt_prefix("ac");
  node.set_master("master");

  node.onConnect([]() {
    Log.println("Connected");
    if (firstOnConnectTime == true) {
      _OvenController->setUserIsApproved(false);
      firstOnConnectTime = false;
      machinestate = WAITINGFORCARD;
    }
  });
  node.onDisconnect([]() {
    Log.println("Disconnected");
//    machinestate = NOCONN;
  });
  node.onError([](acnode_error_t err) {
    Log.printf("Error %d\n", err);
    _OvenController->setUserIsApproved(false);
    machinestate = WAITINGFORCARD;
  });
  node.onApproval([](const char * machine) {
    char resultStr2[SERIAL_MAXLINELENGTH];
    Debug.print("Got approve for machine: ");
    Debug.println(machine);
    if (machinestate == WAITINGFORCARD) {
      _OvenController->setUserIsApproved(true);
      sprintf(resultStr2, "%d:2", e_Status); // User authenticated
      Serial2.println(resultStr2);

      sprintf(resultStr2, "%d", e_ClearMessage);
      Serial2.println(resultStr2);

      machinestate = APPROVED;
      Log.println("User is approved and is now allowed to switch the oven on");
    }
  });
  node.onDenied([](const char * machine) {
    char resultStr3[SERIAL_MAXLINELENGTH];
    Debug.println("Got denied");
    if (machinestate > REJECTED) {
      Debug.println("Denied ingnored, oven is already in use");
    } else {
      machinestate = REJECTED;
      _OvenController->setUserIsApproved(false);
      sprintf(resultStr3, "%d:3", e_Status);  // User denied
      Serial2.println(resultStr3);
    }
  });

  node.set_report_period(20 * 1000);
  node.onReport([](JsonObject  & report) {
    report["state"] = state[machinestate].label;

    if (_OvenController->getAllowPidControllerIsOn()) {
      report["ovencontroller"] = "enabled";
    } else {
      report["ovencontroller"] = "disabled";
    }

    if (_OvenController->getSSRIsOn()) {
      report["Solid State Relais"] = "switched on";
    } else {
      report["Solid State Relais"] = "switched off";
    }

    _OvenController->measureOvenTemps();
    if (_OvenController->getTempFault()) {
      report["Thermocouple"] = "error detected";
    } else {
      report["Thermocouple"] = "working OK";
      if (_OvenController->getValidTemps()) {
        sprintf(reportStr, "Internal temperature node = %6.1f degrees Celcius", _OvenController->getInternalTemp());
        report["Thermocouple"] = reportStr;
        sprintf(reportStr, "Oven temperature = %6.1f degrees Celcius", _OvenController->getThermoCoupleTemp());
        report["Thermocouple"] = reportStr;
      } else {
        report["Thermocouple"] = "No valid temperature readings available";
      }
    }

#ifdef OTA_PASSWD
    report["ota"] = true;
#else
    report["ota"] = false;
#endif
  });

  reader.onSwipe([](const char * tag) -> ACBase::cmd_result_t {

    // avoid swithing messing with the swipe process
    if (machinestate > CHECKINGCARD) {
      Debug.printf("Ignoring a normal swipe - as we're still in some open process.");
      return ACBase::CMD_CLAIMED;
    }

    // We'r declining so that the core library handle sending
    // an approval request, keep state, and so on.
    //
    Debug.printf("Detected a normal swipe.\n");
  //  buzz = CHECK;
    return ACBase::CMD_DECLINE;
  });


  // This reports things such as FW version of the card; which can 'wedge' it. So we
  // disable it unless we absolutely positively need that information.
  //
  reader.set_debug(false);
  node.addHandler(&reader);
 
#ifdef OTA_PASSWD
  node.addHandler(&ota);
#endif

  Log.addPrintStream(std::make_shared<MqttLogStream>(mqttlogStream));

  auto t = std::make_shared<TelnetSerialStream>(telnetSerialStream);
  Log.addPrintStream(t);
  Debug.addPrintStream(t);

  // node.set_debug(true);
  // node.set_debugAlive(true);

// if Olimex ESP32-PoE board is used
#ifdef ESP32_PoE  
  node.begin(BOARD_OLIMEX);
#endif

// if default, board (POESP, board Aart) is used
#ifndef ESP32_PoE
  node.begin();
#endif
  // if FAN_PIN uses gpio0, this port must be (re-)initialized here, because 
  // in the ethernet settings for the ethernet port in node.begin() gpio0 is used.
  // This switches this port to another mode. This use is not needed, because the PHY
  // for the internet port used on the ESP32-PoE is not the same as the one used in
  // the init procedure
  pinMode(FAN_PIN, OUTPUT);
  digitalWrite(FAN_PIN, 0);
  
  _OvenController->addToWebServer(myWebServer, "/", ROOTPAGE);
  _OvenController->addToWebServer(myWebServer, "/prev_schedule_page", PREVPAGE);
  _OvenController->addToWebServer(myWebServer, "/next_schedule_page", NEXTPAGE);
  _OvenController->addToWebServer(myWebServer, "/prev_select_schedule_page", PREVSELECTPAGE);
  _OvenController->addToWebServer(myWebServer, "/next_select_schedule_page", NEXTSELECTPAGE);
//  _OvenController->addToWebServer(myWebServer, "/select_schedule_page", SELECTSCHEDULEPAGE);
  _OvenController->addToWebServer(myWebServer, "/edit_schedules_page", EDITSCHEDULESPAGE);
  _OvenController->addToWebServer(myWebServer, "/action_page", ACTIONPAGE);
#ifdef DEBUGIT
  _OvenController->addToWebServer(myWebServer, "/switch_oven_on_page", SWITCHOVENONPAGE);
  _OvenController->addToWebServer(myWebServer, "/switch_oven_off_page", SWITCHOVENOFFPAGE);
#endif

  myWebServer.onNotFound(handleNotFound);        // When a client requests an unknown URI (i.e. something other than "/"), call function "handleNotFound"
  myWebServer.begin();
  Log.println("Webserver started");

  sprintf(tmpStr, "%d:0", e_Status); // clear status
  Serial2.println(tmpStr);
  sprintf(tmpStr, "%d", e_IsOff);

  double currentOvenTemp = _OvenController->measureOvenTemps();
  if (currentOvenTemp > -300) {
    sprintf(tmpStr, "%d:%d", e_OvenTemp, (int)currentOvenTemp);
    Serial2.println(tmpStr);
  }

  Log.println("Booted: " __FILE__ " " __DATE__ " " __TIME__ );
}

void test_Loop() {
  int tmp = digitalRead(KWH_METER_PIN);
  if (tmp != inputIsHigh) {
    inputIsHigh = tmp;
    digitalWrite(SOLID_STATE_RELAIS_PIN, inputIsHigh);
    digitalWrite(SIGNAL_LAMP_PIN, inputIsHigh);
    digitalWrite(FAN_PIN, inputIsHigh);
  }
}

unsigned long startTime = 0;
int test_State = 0;
int state_duration = 60000; // i minute

void test_Loop_PID() {
  if (startTime == 0) {
    startTime = millis();
    _OvenController->setControllerOn();
    _OvenController->setGoalOvenTemp(30, false);
    Log.println("PID test started (state 0): goal = 30 degrees Celsius");
  }
  if ((millis() - startTime) > state_duration) {
    test_State++;
    if (test_State > 3) {
      test_State = 0;
    }
    switch (test_State)
    {
    case 0:
      _OvenController->setControllerOn();
      _OvenController->setGoalOvenTemp(30, false);
      Log.println("PID test state 0: goal = 30 degrees Celsius");
      break;
    case 1:
      _OvenController->setGoalOvenTemp(200, false);
      Log.println("PID test state 1: goal = 30 degrees Celsius");
      break;
    case 2:
      _OvenController->setGoalOvenTemp(200, false);
      Log.println("PID test state 2: goal = 200 degrees Celsius");
      break;
    case 3:
      _OvenController->setGoalOvenTemp(1000, false);
      Log.println("PID test state 3: goal = 1000 degrees Celsius");
      break;
    default:
      break;
    }
    startTime = millis();
  }

}

void loop() {
  node.loop();

  readSerial2();

  displayLoop();

  myWebServer.handleClient(); 
   
  _OvenController->PIDloop();
  // test_Loop_PID();

  _OvenController->scheduleLoop();
  
  _OvenController->checkTemps();

  countkWhPulses();

  // test_Loop();

  if (laststate != machinestate) {
    Debug.printf("Changed from state <%s> to state <%s>\n",
                 state[laststate].label, state[machinestate].label);

    state[laststate].timeInState += (millis() - laststatechange) / 1000;
    laststate = machinestate;
    laststatechange = millis();
    return;
  }

  if (state[machinestate].maxTimeInMilliSeconds != NEVER &&
      ((millis() - laststatechange) > state[machinestate].maxTimeInMilliSeconds))
  {
    state[machinestate].timeoutTransitions++;

    laststate = machinestate;
    machinestate = state[machinestate].failStateOnTimeout;

    Log.printf("Time-out; transition from <%s> to <%s>\n",
               state[laststate].label, state[machinestate].label);
    return;
  };

  if (state[machinestate].autoReportCycle && \
      (millis() - laststatechange) > state[machinestate].autoReportCycle && \
      (millis() - lastReport) > state[machinestate].autoReportCycle)
  {
    Log.printf("State: %s now for %lu seconds", state[laststate].label, (millis() - laststatechange) / 1000);
    lastReport = millis();
  };

  // aartLed.set(state[machinestate].ledState);

  switch (machinestate) {
    case REBOOT:
      node.delayedReboot();
      break;

    case WAITINGFORCARD:
    case CHECKINGCARD:
      break;
    case CLEARSTATUS:
      sprintf(tmpStr, "%d:0", e_Status); // Clear status
      Serial2.println(tmpStr);
      sprintf(tmpStr, "%d", e_ClearMessage);
      Serial2.println(tmpStr);
      machinestate = WAITINGFORCARD;
      break;
    case REJECTED:
      break;
    case APPROVED:
      if (_OvenController->ovenIsSwitchedOn()) {
        _OvenController->setUserIsApproved(false);
        sprintf(tmpStr, "%d:0", e_Status); // Clear status
        Serial2.println(tmpStr);
        machinestate = OVENON;
        sprintf(tmpStr, "%d", e_IsOn);
        Serial2.println(tmpStr);
        updateDisplayBottom();
        wattPulses = 0;
      }
      break;
    case OVENON:
      // wait until schedule is ready
      if (_OvenController->ovenIsSwitchedOff()) {
        _OvenController->setUserIsApproved(false);
        machinestate = WAITINGFORCARD;
        sprintf(tmpStr, "%d", e_IsOff);
        Serial2.println(tmpStr);
        clearMessageSent = false;
        updateDisplayBottom();
        Log.printf("Count watt pulses = %lu\n\r", wattPulses);
        Log.printf("Oven schedule ready, used energy = %7.3f kWh\n\r", (double)wattPulses / 1000);
      }
      break;
    case OVENOFF:
      _OvenController->switchOvenOff();
      _OvenController->setUserIsApproved(false);
      Log.printf("Count watt pulses = %lu\n\r", wattPulses);
      Log.printf("Oven schedule ready, used energy = %7.3f kWh\n\r", (double)wattPulses / 1000);
      sprintf(tmpStr, "%d", e_IsOff);
      Serial2.println(tmpStr);
      clearMessageSent = false;
      updateDisplayBottom();
      machinestate = WAITINGFORCARD;
      break;
    case BOOTING:
    case OUTOFORDER:
    case TRANSIENTERROR:
    case NOCONN:
      break;
  };
}





