/*
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

#include "pid.h"

#include <Arduino.h>
#include <ACNode.h>

PIDController::PIDController() {
    return;
}

void PIDController::begin(int SSRPin, int FANPin, int LAMPPin, int OutputPowerRange, double initialOvenTemp) {

    SSRPinUsed = SSRPin;
    SSRPowerRange = OutputPowerRange;
    FANPinUsed = FANPin;
    LAMPPinUsed = LAMPPin;

    pinMode(SSRPinUsed, OUTPUT);
    digitalWrite(SSRPinUsed, LOW);

    pinMode(FANPinUsed, OUTPUT);
    digitalWrite(FANPinUsed, LOW);

    pinMode(LAMPPinUsed, OUTPUT);
    digitalWrite(LAMPPinUsed, LOW);

    allowPIDControllerIsOn = false;

    _thermoCouple = new Adafruit_MAX31856(SPI_CS, SPI_DI, SPI_DO, SPI_CLK);

    if (!_thermoCouple->begin()) {
        Log.println("Could not initialize thermocouple.");
    } else {
        Log.println("Thermo couple init OK!");
    }
    
    _thermoCouple->setThermocoupleType(THERMOCOUPLE_TYPE);

    Log.print("Thermocouple type: ");
    switch (_thermoCouple->getThermocoupleType() ) {
        case MAX31856_TCTYPE_B: 
            Log.println("B Type"); 
            break;
        case MAX31856_TCTYPE_E: 
            Log.println("E Type"); 
            break;
        case MAX31856_TCTYPE_J: 
            Log.println("J Type"); 
            break;
        case MAX31856_TCTYPE_K: 
            Log.println("K Type"); 
            break;
        case MAX31856_TCTYPE_N: 
            Log.println("N Type"); 
            break;
        case MAX31856_TCTYPE_R: 
            Log.println("R Type"); 
            break;
        case MAX31856_TCTYPE_S: 
            Log.println("S Type"); 
            break;
        case MAX31856_TCTYPE_T: 
            Log.println("T Type"); 
            break;
        case MAX31856_VMODE_G8: 
            Log.println("Voltage x8 Gain mode"); 
            break;
        case MAX31856_VMODE_G32: 
            Log.println("Voltage x8 Gain mode"); 
            break;
        default: 
            Log.println("Unknown"); 
            break;
        checkTempStartTime = millis();
    }

    initSchedules();
    loadAllSchedules();

    _SSRPID = new PID(&currentOvenTemp, &outputPower, &setPointTemp, 2, 5, 1, DIRECT);

    pulseStartTime = millis();

    setPointTemp = initialOvenTemp;

    _SSRPID->SetOutputLimits(0, (double) OutputPowerRange);

    _SSRPID->SetMode(AUTOMATIC);

    _SSRPID->SetSampleTime(1000);

    switchOvenOff();
}

void PIDController::PIDloop() {
    if (allowPIDControllerIsOn) {
        if (getNewTemp && ((millis() - startSampleTime) > PID_SAMPLE_TIME)) {
            startSampleTime = millis();
            currentOvenTemp = measureOvenTemps();
            getNewTemp = false;
        } else {
            currentOvenTemp = currentThermoCoupleTemp;
        }

        if (currentOvenTemp > -300) {
            if (_SSRPID->Compute()) {
                getNewTemp = true;
                startSampleTime = millis();
#ifdef DEBUGIT                
                Serial.printf("OutputPower = %5.0f ms currentOvenTemp = %6.1f, goal = %6.1f\n\r", outputPower, currentOvenTemp, setPointTemp);
#endif                
            }
        
            now = millis();

            if ((now - pulseStartTime) > SSRPowerRange) {
                pulseStartTime += SSRPowerRange;
            }

            if ((now - pulseStartTime) < outputPower) {
                if (!SSRIsOn) {
                    digitalWrite(SSRPinUsed, HIGH);
                    SSRIsOn = true;
                }
            } else {
                if (SSRIsOn) {
                    digitalWrite(SSRPinUsed, LOW);
                    SSRIsOn = false;
                }
            }
        } else {
            if (tempFault) {
                // switch off SSR to prevent overheating
                if (SSRIsOn) {
                    digitalWrite(SSRPinUsed, LOW);
                    SSRIsOn = false;
                }
                allowPIDControllerIsOn = false;
            }
        }
    }
}

void PIDController::setControllerOn() {
    allowPIDControllerIsOn = true;
    pulseStartTime = millis();
}

void PIDController::setControllerOff() {
    allowPIDControllerIsOn = false;
    digitalWrite(SSRPinUsed, LOW);
    SSRIsOn = false;
}

void PIDController::setGoalOvenTemp(double NewOvenTemp, bool ignoreNegativeValue) {
    if ((NewOvenTemp < setPointTemp) && !ignoreNegativeValue) {
        _SSRPID->SetOutputLimits(0, 0.0001);
        _SSRPID->SetOutputLimits(0, (double) SSRPowerRange);
    }
    setPointTemp = NewOvenTemp;
    outputPower = 0;
}

double PIDController::measureOvenTemps() {
    currentInternalTemp = (double)_thermoCouple->readCJTemperature();
    currentThermoCoupleTemp = (double)_thermoCouple->readThermocoupleTemperature();

    // Check and print any faults
    fault = _thermoCouple->readFault();
    if (fault) {
        validTemps = false;
        if (prevFault != fault) {
            if (fault & MAX31856_FAULT_CJRANGE) {
                Log.println("Error Thermocouple: Cold Junction Range Fault");
            }
            if (fault & MAX31856_FAULT_TCRANGE) {
                Log.println("Error Thermocouple: Thermocouple Range Fault");
            }
            if (fault & MAX31856_FAULT_CJHIGH) {
                Log.println("Error Thermocouple: Cold Junction High Fault");
            }
            if (fault & MAX31856_FAULT_CJLOW) {
                Log.println("Error Thermocouple: Cold Junction Low Fault");
            }
            if (fault & MAX31856_FAULT_TCHIGH) {
                Log.println("Error Thermocouple: Thermocouple High Fault");
            }
            if (fault & MAX31856_FAULT_TCLOW) {
                Log.println("Error Thermocouple: Thermocouple Low Fault");
            }
            if (fault & MAX31856_FAULT_OVUV) {
                Log.println("Error Thermocouple: Over/Under Voltage Fault");
            }
            if (fault & MAX31856_FAULT_OPEN) {
                Log.println("Error Thermocouple: Thermocouple Open Fault");
            }
        }
        prevFault = fault;
        if (tempFaultCount < MAX_TEMPFAULTS) {
            tempFaultCount++;
        } else {
            tempFault = true;
        }
        return (double) -300;
    } else {
        validTemps = true;
        tempFaultCount = 0;
        if (prevFault != 0) {
            Log.println("Thermocouple: error solved");
        }
        tempFault = false;
        prevFault = 0;
        return (double) currentThermoCoupleTemp;
    }
}

bool PIDController::getValidTemps() {
    return validTemps;
}

bool PIDController::getTempFault() {
    return tempFault;
}

double PIDController::getInternalTemp() {
    return currentInternalTemp;
}

double PIDController::getThermoCoupleTemp() {
    return currentThermoCoupleTemp;
}

bool PIDController::getAllowPidControllerIsOn() {
    return allowPIDControllerIsOn;
}

bool PIDController::getSSRIsOn() {
    return SSRIsOn;
}

int PIDController::getSelectedSchedule() {
    return selectedSchedule;
}

String PIDController::getScheduleName() {
    return (String)allSchedules[selectedSchedule].scheduleName;
}

bool PIDController::getScheduleIsEmpty() {
    bool scheduleIsEmpty = true;
    int i;

    i = 0;
    while (scheduleIsEmpty && (i < MAX_POINTS_PER_SCHEDULE)) {
        if (allSchedules[selectedSchedule].schedulePoint[i].segmentIsUsed) {
            scheduleIsEmpty = false;
        } else {
            i++;
        }
    }
    return scheduleIsEmpty;
}


int PIDController::getCurrentSegment() {
    return currentPoint;
}

String PIDController::getCurrentMode() {
    switch (allSchedules[selectedSchedule].schedulePoint[currentPoint].tempMode) {
    case SWITCHED_OFF:
        return "SWITCHED_OFF";
        break;
    case HOLD:
        return "HOLD";
        break;
    case RAMP:
        return "RAMP";
        break;
    }
    return "UNKNOWN";
}

double PIDController::getCurrentGoal() {
    return allSchedules[selectedSchedule].schedulePoint[currentPoint].tempGoal;
}

unsigned long PIDController::getTimeLeft() {
    return ((unsigned long)allSchedules[selectedSchedule].schedulePoint[currentPoint].timeToNextPoint * 60) - (millis() - segmentStartTime) / 1000;
}

void PIDController::initSchedules() {
    for (int i = 0; i < MAX_NR_OF_SCHEDULES; i++) {
        allSchedules[i].scheduleName[0] = 0;
    }
}

bool PIDController::saveSingleSchedule(int scheduleToSave) {
    char s2[64];
    sprintf(s2, "%d", scheduleToSave);
    String path = SCHEDULES_DIR_PREFIX + (String)SCHEDULES_FILE_PREFIX + (String)s2;
    File schedulesFile;
    unsigned int writeSize;
    schedulesFile = SPIFFS.open(path, "wb");
    if(!schedulesFile) {
      Log.print("There was an error opening the ");
      Log.print(SCHEDULES_DIR_PREFIX);
      Log.print(SCHEDULES_FILE_PREFIX);
      Log.print(scheduleToSave);
      Log.println(" file for writing");
      return false;
    }
    writeSize = schedulesFile.write((byte*)&allSchedules[scheduleToSave], sizeof(allSchedules[scheduleToSave]));
    schedulesFile.close();
    if (writeSize != sizeof(allSchedules[scheduleToSave])) {
      Log.printf("ERROR --> schedule %d is NOT stored in SPIFFS\n\r", scheduleToSave);
      return false;
    }
    return true;
}


void PIDController::loadAllSchedules() {
    for (int i = 0; i < MAX_NR_OF_SCHEDULES; i++) {
        loadSingleSchedule(i);
    }
}

void PIDController::loadSingleSchedule(int scheduleToLoad) {
    char s2[64];
    sprintf(s2, "%d", scheduleToLoad);
    String path = SCHEDULES_DIR_PREFIX + (String)SCHEDULES_FILE_PREFIX + (String)s2;
    File scheduleFile;
    unsigned int readSize;

    if(!SPIFFS.begin(false)){
        Log.println("An Error has occurred while mounting SPIFFS");
        scheduleIsLoaded = false;
        return;
    }
    if (SPIFFS.exists(path)) {
        scheduleFile = SPIFFS.open(path, "rb");
        if(!scheduleFile) {
            Log.print("There was an error opening the ");
            Log.print(SCHEDULES_DIR_PREFIX);
            Log.print(SCHEDULES_FILE_PREFIX);
            Log.print(scheduleToLoad);
            Log.println(" file for reading");
            scheduleIsLoaded = false;
            return;
        }
        scheduleFile.setTimeout(0);
        readSize = scheduleFile.readBytes((char*)&allSchedules[scheduleToLoad], sizeof(allSchedules[scheduleToLoad]));
        scheduleFile.close();
        if (readSize != sizeof(allSchedules[scheduleToLoad])) {
            Log.print("There was an error reading schedule from ");
            Log.print(SCHEDULES_DIR_PREFIX);
            Log.println(SCHEDULES_FILE_PREFIX);
            Log.print(scheduleToLoad);
            scheduleIsLoaded = false;
            return;
        } else {
            currentSchedule = 0;
            scheduleIsLoaded = true;
            return;
        }

    } else {
        Log.print("The file ");
        Log.print(SCHEDULES_DIR_PREFIX);
        Log.print(SCHEDULES_FILE_PREFIX);
        Log.print(scheduleToLoad);
        Log.println(" is not available!");
        scheduleIsLoaded = false;
        return;
    }
}

void PIDController::addToWebServer(ESP32WebServer &theWebServer, const char *URI, webPage_t webPage) {
    switch (webPage) {
    case ROOTPAGE:
        theWebServer.on(URI, std::bind(&PIDController::handleRootWebPage, this));
        break;
    case PREVPAGE:
        theWebServer.on(URI, std::bind(&PIDController::handlePrevWebPage, this));
        break;
    case NEXTPAGE:
        theWebServer.on(URI, std::bind(&PIDController::handleNextWebPage, this));
        break;
    case PREVSELECTPAGE:
        theWebServer.on(URI, std::bind(&PIDController::handlePrevSelectWebPage, this));
        break;
    case NEXTSELECTPAGE:
        theWebServer.on(URI, std::bind(&PIDController::handleNextSelectWebPage, this));
        break;
    case EDITSCHEDULESPAGE:
        theWebServer.on(URI, std::bind(&PIDController::editSchedulesWebPage, this));
        break;
    case ACTIONPAGE:
        theWebServer.on(URI, std::bind(&PIDController::handleActionWebPage, this));
        break;
#ifdef DEBUGIT        
    case SWITCHOVENONPAGE:
        theWebServer.on(URI, std::bind(&PIDController::handleSwitchOvenOnPage, this));
        break;
    case SWITCHOVENOFFPAGE:
        theWebServer.on(URI, std::bind(&PIDController::handleSwitchOvenOffPage, this));
        break;
#endif        
    default:
        break;
    }
    this->webServer = &theWebServer;
}

String PIDController::formLine_intParam(const char * labelText, const char * paramName, int paramID, int paramValue, int minvalue, int maxvalue, char * unit)
{
	char limits[512];
    char resultStr[512];

	sprintf(limits, "(%d - %d)", minvalue, maxvalue);

	if (strlen(unit) <= 0)
	{
		sprintf(resultStr, "<label>%s %s: </label><input type='number' name='%s_%d' min='%d' max='%d' size='10' value='%d'>\n", labelText, limits, paramName, paramID, minvalue, maxvalue, paramValue);
	}
	else
	{
		sprintf(resultStr, "<label>%s %s %s: </label><input type='number' name='%s_%d' min='%d' max='%d' size='10' value='%d'>\n", labelText, limits, unit, paramName, paramID, minvalue, maxvalue, paramValue);
	}
	return (String) resultStr;
}

String PIDController::formLine_boolParam(const char * labelText, const char * paramName, int paramID, bool paramValue)
{
	char resultStr[512];

	if (paramValue == true)
	{
		sprintf(resultStr, "<label>%s: </label><input type='checkbox' id='%s_%d' name='%s_%d' checked>\n", labelText, paramName, paramID, paramName, paramID);
	}
	else
	{
		sprintf(resultStr, "<label>%s: </label><input type='checkbox' id='%s _%d' name='%s_%d'>\n", labelText, paramName, paramID, paramName, paramID);
	}
    return (String) resultStr;
}

void PIDController::showSelectedSchedule(String &s) {

    char s2[128];

    bool scheduleIsEmpty = true;

    s = s + F("<br>\n");
    s = s + F("<table>\n");
    for (int i = 0; i < MAX_POINTS_PER_SCHEDULE; i++) {
        if (allSchedules[selectedSchedule].schedulePoint[i].segmentIsUsed) {
            scheduleIsEmpty = false;
            s = s + F("<tr>\n");
            s = s + F("<td>\n");
            sprintf(s2, "<b>Segment %d: </b>\n", i + 1);
            s = s + (String)s2;
            s = s + F("</td>\n");
            s = s + F("<td>\n");
            switch (allSchedules[selectedSchedule].schedulePoint[i].tempMode) {
            case SWITCHED_OFF:
                sprintf(s2, "<b>Temp. controller mode:</b> SWITCHED OFF\n");
                break;
            case HOLD:
                sprintf(s2, "<b>Temp. controller mode:</b> HOLD\n");
                break;
            case RAMP:
                sprintf(s2, "<b>Temp. controller mode:</b> RAMP\n");
                break;
            }
            s = s + (String)s2;
            s = s + F("</td>\n");
            s = s + F("<td>\n");
            sprintf(s2, "<b>Temperature goal:</b> %d &degC\n", (int)allSchedules[selectedSchedule].schedulePoint[i].tempGoal);
            s = s + String(s2);
            s = s + F("</td>\n");
            s = s + F("<td>\n");
            sprintf(s2, "<b>Segment duration:</b> %d min.\n", allSchedules[selectedSchedule].schedulePoint[i].timeToNextPoint);
            s = s + (String)s2;
            s = s + F("</td>\n");
            s = s + F("</tr>\n");
        }
    }
    s = s + F("</table>");
    if (scheduleIsEmpty) {
        s = s + F("<b>Schedule is empty!</b>\n");
    }
    s = s + F("<br>\n");
}

void PIDController::editSchedulesWebPage() {
    String s;

    char s2[128];

    if (ovenIsOn) {
        handleRootWebPage();
        return;
    }

    if (webServer != nullptr) {
        s = F("<!DOCTYPE html>\n");
        s = s + F("<html>\n");
        s = s + F("<body>\n");
        s = s + F("<style> table { width:80%%; } </style>\n");
        s = s + F("<h2>Ceramic Oven Controller MakerSpace Leiden<h2>\n");
        s = s + F("<h3>Change Oven schedules</h3>\n");
        s = s + F("<form action=\"/action_page\">\n");
        s = s + F("<br>\n");
        sprintf(s2, "<b>Warning: independent of the configured total duration of a schedule, the oven always switches off after: %d hours</b>\n", MAX_OVEN_ON_TIME);
        s = s + (String)s2;
        s = s + F("<br>\n");
        s = s + F("<br>\n");
        s = s + F("<table>\n");
        s = s + F("<tr>\n");
        s = s + F("<td>\n");
        s = s + F("<input type=\"submit\" formaction=\"/prev_schedule_page\" value=\"prev schedule\">\n");
        s = s + F("</td>\n");
        s = s + F("<td>\n");
        s = s + F("<input type=\"submit\" formaction=\"/next_schedule_page\" value=\"next schedule\">\n");
        s = s + F("</td>\n");
        s = s + F("<td>\n");
        s = s + F("<input type=\"submit\" formaction=\"/edit_schedules_page\" value=\"Cancel changes made\">\n");
        s = s + F("</td>\n");
        s = s + F("<td>\n");
        s = s + F("<input type=\"submit\" value=\"Save changes\">\n");
        s = s + F("</td>\n");
        s = s + F("<td>\n");
        s = s + F("<input type=\"submit\" formaction=\"/\" value=\"home page\">\n");
        s = s + F("</td>\n");
        s = s + F("</tr>\n");
        s = s + F("</table>\n");
        s = s + F("<br>\n");
        s = s + F("<br>\n");
        sprintf(s2, "<b>Current schedule: %d</b>\n", currentSchedule + 1);
        s = s + (String)s2;
        s = s + F("<br>\n");
        s = s + F("<br>\n");
        sprintf(s2, "<b>Schedule name: </b> <input type=\"text\" name=\"scheduleName\" value=\"%s\">\n", allSchedules[currentSchedule].scheduleName);
        s = s + (String)s2;
        s = s + F("<br>\n");
        s = s + F("<br>\n");
        s = s + F("<table>\n");
        for (int i = 0; i < MAX_POINTS_PER_SCHEDULE; i++) {
            s = s + F("<tr>\n");
            s = s + F("<td>\n");
            sprintf(s2, "<b>Segment %d: </b>\n", i + 1);
            s = s + (String)s2;
            s = s + F("</td>\n");
            s = s + F("<td>\n");
            sprintf(s2, "<b>Temp. controller mode: </b> <select id=\"controller_mode_%d\" name=\"mode_%d\">\n", i, i);
            s = s + (String)s2;
            if (allSchedules[currentSchedule].schedulePoint[i].tempMode == SWITCHED_OFF) {
                s = s + F("<option value=\"switched_off\" selected>Switched Off</option>\n");
            } else {
                s = s + F("<option value=\"switched_off\">Switched Off</option>\n");
            }
            if (allSchedules[currentSchedule].schedulePoint[i].tempMode == HOLD) {
                s = s + F("<option value=\"hold\" selected>Hold</option>\n");
            } else {
                s = s + F("<option value=\"hold\">Hold</option>\n");
            }
            if (allSchedules[currentSchedule].schedulePoint[i].tempMode == RAMP) {
                s = s + F("<option value=\"ramp\" selected>Ramp</option>\n");
            } else {
                s = s + F("<option value=\"ramp\">Ramp</option>\n");
            }
            s = s + F("</select>\n");
            s = s + F("</td>\n");
            s = s + F("<td>\n");
            sprintf(s2, " &degC");
            s = s + formLine_intParam("Temperature goal", "tempGoal", i, (int)allSchedules[currentSchedule].schedulePoint[i].tempGoal, MIN_OVEN_TEMP, MAX_OVEN_TEMP, s2);
            s = s + F("</td>\n");
            s = s + F("<td>\n");
            sprintf(s2, " min.");
            s = s + formLine_intParam("Segment duration", "timeToNextPoint", i, allSchedules[currentSchedule].schedulePoint[i].timeToNextPoint, MIN_SEGMENT_TIME, MAX_SEGMENT_TIME, s2);
            s = s + F("</td>\n");
            s = s + F("<td>\n");
            s = s + formLine_boolParam("Segment is used", "segmentIsUsed", i, allSchedules[currentSchedule].schedulePoint[i].segmentIsUsed);
            s = s + F("</td>\n");
            s = s + F("</tr>\n");
        }

        s = s + F("</form>\n");
        s = s + F("</body>\n");
        s = s + F("</html>\n");
        webServer->send(200, "text/html", s);
    }
}

void PIDController::handleRootWebPage() {
    String s;

    char s2[128];

    if (webServer != nullptr) {
        s = F("<!DOCTYPE html>\n");
        s = s + F("<html>\n");
        if (ovenIsOn) {
            s = s + F("<head>\n");
            s = s + F("<meta http-equiv=\"refresh\" content=\"1;url=/\"> \n");
            s = s + F("</head>\n");
        } else {
            s = s + F("<head>\n");
            s = s + F("<meta http-equiv=\"refresh\" content=\"1;url=/\"> \n");
            s = s + F("</head>\n");
        }
        s = s + F("<body>\n");
        s = s + F("<style> table { width:80%%; } </style>\n");
        s = s + F("<h2>Ceramic Oven Controller MakerSpace Leiden<h2>\n");
        s = s + F("<h3>Home page</h3>\n");
        s = s + F("<form action=\"/action_page\">\n");
        s = s + F("<br>\n");
        sprintf(s2, "<b>Warning: independent of the configured total duration of a schedule, the oven always switches off after: %d hours</b>\n", MAX_OVEN_ON_TIME);
        s = s + (String)s2;
        s = s + F("<br>\n");
        s = s + F("<br>\n");
        s = s + F("<table>\n");
        s = s + F("<tr>\n");
        s = s + F("<td>\n");
        if (!ovenIsOn) {
            s = s + F("<input type=\"submit\" formaction=\"/edit_schedules_page\" value=\"edit schedule(s)\">\n");
        } else {
            s = s + F("<input type=\"button\" formaction=\"/edit_schedules_page\" value=\"edit schedule(s)\">\n");
        }
        s = s + F("</td>\n");
        s = s + F("</tr>\n");
        s = s + F("</table>\n");
        s = s + F("<br>\n");
#ifdef DEBUGIT
        s = s + F("<br>\n");
        s = s + F("<b><u>Please authenticate first (with RFId card or tag) before switching on the oven</u></b>\n");
        s = s + F("<br>\n");
        s = s + F("<br>\n");
        s = s + F("<table>\n");
        s = s + F("<tr>\n");
        s = s + F("<td>\n");
        s = s + F("<input type=\"submit\" formaction=\"/switch_oven_off_page\" value=\"Switch oven OFF\">\n");
        s = s + F("</td>\n");
        s = s + F("<td>\n");
        if (!ovenIsOn) {
            s = s + F("<input type=\"submit\" formaction=\"/switch_oven_on_page\" value=\"Switch oven ON\">\n");
        } else {
            s = s + F("<input type=\"button\" formaction=\"/switch_oven_on_page\" value=\"Switch oven ON\">\n");
        }
        s = s + F("</td>\n");
        s = s + F("</tr>\n");
        s = s + F("</table>\n");
        s = s + F("<br>\n");
#endif
        s = s + F("<br>\n");
        s = s + F("<table>\n");
        s = s + F("<tr>\n");
        s = s + F("<td>\n");
        if (!ovenIsOn) {
            s = s + F("<input type=\"submit\" formaction=\"/prev_select_schedule_page\" value=\"prev schedule\">\n");
        } else {
            s = s + F("<input type=\"button\" formaction=\"/prev_select_schedule_page\" value=\"prev schedule\">\n");
        }
        s = s + F("</td>\n");
        s = s + F("<td>\n");
        if (!ovenIsOn) {
            s = s + F("<input type=\"submit\" formaction=\"/next_select_schedule_page\" value=\"next schedule\">\n");
        } else {
            s = s + F("<input type=\"button\" formaction=\"/next_select_schedule_page\" value=\"next schedule\">\n");
        }
        s = s + F("</td>\n");
        s = s + F("</tr>\n");
        s = s + F("</table>\n");
        s = s + F("<br>\n");
        s = s + F("<br>\n");
        s = s + F("<b>Schedule:</b>\n");
        s = s + F("<br>\n");
        sprintf(s2, "selected schedule = %d\n", selectedSchedule + 1);
        s = s + (String)s2;
        s = s + F("<br>\n");
        sprintf(s2, "Schedule name = %s\n", allSchedules[selectedSchedule].scheduleName);
        s = s + (String)s2;
        s = s + F("<br>\n");
        showSelectedSchedule(s);
        s = s + F("<br>\n");
        s = s + F("<b>Oven:</b>\n");
        if (ovenIsOn) {
            s = s + F("Oven is switched on\n");
        } else {
            s = s + F("Oven is switched off\n");
        }
        s = s + F("<br>\n");
        if ((measureOvenTemps() > -300) && !getTempFault()) {
            sprintf(s2, "Internal temp. = %d &degC\n", (int)currentInternalTemp);
            s = s + (String)s2;
            s = s + F("<br>\n");
            sprintf(s2, "Oven temp.     = %d &degC\n", (int)currentThermoCoupleTemp);
            s = s + (String)s2;
            s = s + F("<br>\n");
        } else {
            s = s + (F("<b>Thermocouple fault, oven will not start until this issue is solved!\n</b>"));
            s = s + F("<br>\n");
        }
        if (ovenIsOn) {
            s = s + F("<b>Schedule:</b>\n");
            s = s + F("<br>\n");
            sprintf(s2, "Current segment = %d\n", currentPoint + 1);
            s = s + (String)s2;
            s = s + F("<br>\n");
            switch (allSchedules[selectedSchedule].schedulePoint[currentPoint].tempMode) {
                case SWITCHED_OFF:
                    s = s + F("Current mode = SWITCHED_OFF\n");
                break;
                case HOLD:
                    s = s + F("Current mode = HOLD\n");
                break;
                case RAMP:
                    s = s + F("Current mode = RAMP\n");
                break;                
            }
            s = s + F("<br>\n");
            sprintf(s2, "Previous goal= %d &degC\n", (int)previousGoal);
            s = s + (String)s2;
            s = s + F("<br>\n");
            sprintf(s2, "New goal= %d &degC\n", (int)nextGoal);
            s = s + (String)s2;
            s = s + F("<br>\n");
            if (allSchedules[selectedSchedule].schedulePoint[currentPoint].tempMode == RAMP) {
                sprintf(s2, "Current goal= %d &degC\n", (int)rampValue);
                s = s + (String)s2;
                s = s + F("<br>\n");
            }
            sprintf(s2, "Duration  = %d min.\n", allSchedules[selectedSchedule].schedulePoint[currentPoint].timeToNextPoint);
            s = s + (String)s2;
            s = s + F("<br>\n");

            time_t t = (allSchedules[selectedSchedule].schedulePoint[currentPoint].timeToNextPoint * 60) - (int)(millis() - segmentStartTime) / 1000;
            tm *local_Time = localtime(&t);

            sprintf(s2, "Time left = %02d:%02d:%02d h", local_Time->tm_hour, local_Time->tm_min, local_Time->tm_sec);
            s = s + (String)s2;
            s = s + F("<br>\n");
        }
        s = s + F("</form>\n");
        s = s + F("</body>\n");
        s = s + F("</html>\n");
        webServer->send(200, "text/html", s);
    }
}

void PIDController::handlePrevWebPage() {
    changeParams();
    if (currentSchedule > 0) {
        currentSchedule--;
    } else {
        currentSchedule = MAX_NR_OF_SCHEDULES - 1;
    }
    editSchedulesWebPage();
}

void PIDController::handleNextWebPage() {
    changeParams();
    if (currentSchedule < (MAX_NR_OF_SCHEDULES - 1)) {
        currentSchedule++;
    } else {
        currentSchedule = 0;
    }
    editSchedulesWebPage();
}

void PIDController::handlePrevSelectWebPage() {
    if (selectedSchedule > 0) {
        selectedSchedule--;
    } else {
        selectedSchedule = MAX_NR_OF_SCHEDULES - 1;
    }
    handleRootWebPage();
}

void PIDController::handleNextSelectWebPage() {
    if (selectedSchedule < (MAX_NR_OF_SCHEDULES - 1)) {
        selectedSchedule++;
    } else {
        selectedSchedule = 0;
    }
    handleRootWebPage();
}

void PIDController::changeParams() {
    String s;

    char s2[128];

    s = webServer->arg("scheduleName");
    strncpy(allSchedules[currentSchedule].scheduleName, s.c_str(), sizeof(allSchedules[currentSchedule].scheduleName));
    allSchedules[currentSchedule].scheduleName[sizeof(allSchedules[currentSchedule].scheduleName) - 1] = 0;
    for (int i = 0; i < MAX_POINTS_PER_SCHEDULE; i++) {
        sprintf(s2, "mode_%d", i);
        if (webServer->arg(s2).equals("switched_off")) {
            allSchedules[currentSchedule].schedulePoint[i].tempMode = SWITCHED_OFF;
        } else {
            if (webServer->arg(s2).equals("hold")) {
                allSchedules[currentSchedule].schedulePoint[i].tempMode = HOLD;
            } else {
                if (webServer->arg(s2).equals("ramp")) {
                    allSchedules[currentSchedule].schedulePoint[i].tempMode = RAMP;
                }
            }
        }
        sprintf(s2, "tempGoal_%d", i);
        s = webServer->arg(s2);
        allSchedules[currentSchedule].schedulePoint[i].tempGoal = s.toDouble();
        sprintf(s2, "timeToNextPoint_%d", i);
        s = webServer->arg(s2);
        allSchedules[currentSchedule].schedulePoint[i].timeToNextPoint = s.toInt();
        sprintf(s2, "segmentIsUsed_%d", i);
        if (webServer->hasArg(s2))
        {
            s = webServer->arg(s2);
            if (s.equals("on")) {
                allSchedules[currentSchedule].schedulePoint[i].segmentIsUsed = true;
            } else {
                allSchedules[currentSchedule].schedulePoint[i].segmentIsUsed = false;
            }

        } else {
            allSchedules[currentSchedule].schedulePoint[i].segmentIsUsed = false;
        }
    }
    saveSingleSchedule(currentSchedule);
}

void PIDController::handleActionWebPage() {
    if (ovenIsOn) {
        handleRootWebPage();
        return;
    }

    changeParams();
    editSchedulesWebPage();
}

#ifdef DEBUGIT
void PIDController::handleSwitchOvenOnPage() {
    if (theUserIsApproved) {        
        switchOvenOn();
        Log.println("Debugging: Oven switched ON via webpage!");
    } else {
        Log.println("Debugging: tried to switch the oven on, via webpage, while the user is not approved (yet)");
    }
    handleRootWebPage();
}

void PIDController::handleSwitchOvenOffPage() {
    switchOvenOff();
    Log.println("Debugging: Oven switched OFF via webpage!");
    handleRootWebPage();
}
#endif

void PIDController::selectMode() {
    segmentStartTime = millis();
    previousGoal = nextGoal;
    nextGoal = allSchedules[selectedSchedule].schedulePoint[currentPoint].tempGoal;
    switch (allSchedules[selectedSchedule].schedulePoint[currentPoint].tempMode) {
        case SWITCHED_OFF:
            nextGoal = 0;
            setGoalOvenTemp(nextGoal, false);
            setControllerOff();
            rampIsOn = false;
            Log.println("Current mode = SWITCHED_OFF");
            return;
        break;
        case HOLD:
            if (!allowPIDControllerIsOn) {
                setControllerOn();
            }
            setGoalOvenTemp(nextGoal, false);
            rampIsOn = false;
            Log.println("Current mode = HOLD");
            return;
        break;
        case RAMP:
            if (!allowPIDControllerIsOn) {
                setControllerOn();
            }
            if (allSchedules[selectedSchedule].schedulePoint[currentPoint].timeToNextPoint < 1) {
                setGoalOvenTemp(nextGoal, false);
                rampIsOn = false;
                Log.println("Current mode = HOLD (implicit: time to next segment = 0)");
                return;
            } else {
                rampValue = previousGoal;
                deltaTemp = (nextGoal - previousGoal) / ((double)allSchedules[selectedSchedule].schedulePoint[currentPoint].timeToNextPoint * 60000 / SCHEDULE_SAMPLE_TIME);
                scheduleStartTime = millis();
                rampIsOn = true;
                Log.printf("Current mode = RAMP (from %d degrees C to %d degrees C)\n\r", (int)previousGoal, (int)nextGoal);
                return;
            }
        break;
    }
}


void PIDController::switchOvenOn() {
    if (theUserIsApproved) {
        if (!ovenIsOn) {
            currentPoint = 0;
            while ((currentPoint < MAX_POINTS_PER_SCHEDULE) && !allSchedules[selectedSchedule].schedulePoint[currentPoint].segmentIsUsed) {
                currentPoint++;
            }
            if (currentPoint < MAX_POINTS_PER_SCHEDULE) {
                Log.println("Ceramic oven is switched on");
                Log.printf("Selected schedule: %d\n\r", selectedSchedule);
                Log.printf("Schedulename: %s\n\r", allSchedules[selectedSchedule].scheduleName);
                Log.printf("Oven schedule starts with segment: %d\n\r", currentPoint);
                previousGoal = 0;
                nextGoal = 0;
                ovenIsOn = true;
                ovenSwitchedOff = false;
                ovenSwitchedOn = true;
                currentPointEndTime = allSchedules[selectedSchedule].schedulePoint[currentPoint].timeToNextPoint * 60000;
                ovenStartTime = millis();
                selectMode();
            } else {
                Log.println("Unable to switch ceramic oven on, due to empty schedule");
            }
        }
    } else {
        Log.println("Tried to switch the oven on, while the user is not approved (yet)");
    }
}

void PIDController::switchOvenOff() {
    setControllerOff();
    ovenIsOn = false;
    ovenSwitchedOn = false;
    ovenSwitchedOff = true;
    Log.println("Ceramic oven is switched off");
}

void PIDController::scheduleLoop() {
    if (ovenIsOn) {
        if (getTempFault()) {
            Log.println("Ceramic oven is switched off, due to error with thermocouple");
            switchOvenOff();
            return;
        }
        if ((millis() - ovenStartTime) > (MAX_OVEN_ON_TIME * 3600000)) {
            Log.printf("Ceramic oven was on for more than %d hour\n\r", MAX_OVEN_ON_TIME);
            switchOvenOff();
            return;
        }
        if ((millis() - ovenStartTime) > currentPointEndTime) {
            if (currentPoint >= (MAX_POINTS_PER_SCHEDULE - 1)) {
                Log.println("Ceramic oven: selected schedule is ready");
                switchOvenOff();
                return;
            }
            currentPoint++;
            while ((currentPoint < MAX_POINTS_PER_SCHEDULE) && !allSchedules[selectedSchedule].schedulePoint[currentPoint].segmentIsUsed) {
                currentPoint++;
            }
            if (currentPoint < MAX_POINTS_PER_SCHEDULE) {
                Log.printf("Next segment (%d) of schedule started\n\r", currentPoint);
                currentPointEndTime = currentPointEndTime + allSchedules[selectedSchedule].schedulePoint[currentPoint].timeToNextPoint * 60000;
                selectMode();
            } else {
                Log.println("Ceramic oven: selected schedule is ready");
                switchOvenOff();
                return;
            }
        }
        if (rampIsOn && (millis() - scheduleStartTime) > SCHEDULE_SAMPLE_TIME) {
            scheduleStartTime = scheduleStartTime + SCHEDULE_SAMPLE_TIME;
            rampValue = rampValue + deltaTemp;
            setGoalOvenTemp(rampValue, true);
            return;
        }
    }
}

void PIDController::setUserIsApproved(bool isApproved) {
    theUserIsApproved = isApproved;
}

bool PIDController::ovenIsSwitchedOn() {
    bool tmpBool = ovenSwitchedOn;

    ovenSwitchedOn = false;
    return tmpBool;
}

bool PIDController::ovenIsSwitchedOff() {
    bool tmpBool = ovenSwitchedOff;

    ovenSwitchedOff = false;
    return tmpBool;
}

void PIDController::checkTemps() {
    if ((millis() - checkTempStartTime) > CHECK_TEMP_SAMPLE_TIME) {
        checkTempStartTime = millis();
        if (measureOvenTemps() > -300) {
            if (getInternalTemp() > FAN_ON_TEMP) {
                if (!FANIsOn) {
                    digitalWrite(FANPinUsed, HIGH);
                    FANIsOn = true;
                    Log.println("Fan is switched on");
                }
            } else {
                if (FANIsOn) {
                    digitalWrite(FANPinUsed, LOW);
                    FANIsOn = false;
                    Log.println("Fan is switched off");
                }
            }
            if (getThermoCoupleTemp() > LAMP_ON_TEMP) {
                if (!LAMPIsOn) {
                    digitalWrite(LAMPPinUsed, HIGH);
                    LAMPIsOn = true;
                    Log.println("Signal lamp is switched on");
                }
            } else {
                if (LAMPIsOn) {
                    digitalWrite(LAMPPinUsed, LOW);
                    LAMPIsOn = false;
                    Log.println("Signal lamp is switched off");
                }
            }
        }
    }

}

void PIDController::selectSchedule(bool nextSchedule) {
    if (nextSchedule) {
        if (selectedSchedule < (MAX_NR_OF_SCHEDULES - 1)) {
            selectedSchedule++;
        } else {
            selectedSchedule = 0;
        }
    } else {
        if (selectedSchedule > 0) {
            selectedSchedule--;
        } else {
            selectedSchedule = MAX_NR_OF_SCHEDULES - 1;
        }
    }
}