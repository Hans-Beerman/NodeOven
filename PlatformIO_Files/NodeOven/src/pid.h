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


#pragma once

// for debugging
// see DEBUGIT flag in platformio.ini

#include <Adafruit_MAX31856.h>
#include <PID_v1.h>
#include <ESP32WebServer.h>

#ifndef SPI_CS
#define SPI_CS                  15
#endif

#ifndef SPI_DI
#define SPI_DI                  5
#endif

#ifndef SPI_DO
#define SPI_DO                  2
#endif

#ifndef SPI_CLK
#define SPI_CLK                 14
#endif

#ifndef THERMOCOUPLE_TYPE
#define THERMOCOUPLE_TYPE       MAX31856_TCTYPE_K
#endif

#ifndef MAX_TEMPFAULTS
#define MAX_TEMPFAULTS          5
#endif

#ifndef PID_SAMPLE_TIME
#define PID_SAMPLE_TIME         500  // in ms
#endif

#ifndef MAX_POINTS_PER_SCHEDULE
#define MAX_POINTS_PER_SCHEDULE 20
#endif

#ifndef MAX_NR_OF_SCHEDULES
#define MAX_NR_OF_SCHEDULES     20
#endif

#ifndef SCHEDULES_DIR_PREFIX
#define SCHEDULES_DIR_PREFIX    "/init"
#endif

#ifndef SCHEDULES_FILE_PREFIX
#define SCHEDULES_FILE_PREFIX   "/schedules"
#endif

#ifndef MIN_OVEN_TEMP
#define MIN_OVEN_TEMP           0
#endif

#ifndef MAX_OVEN_TEMP
#define MAX_OVEN_TEMP           1500
#endif

#ifndef MAX_OVEN_ON_TIME
#define MAX_OVEN_ON_TIME        100 // in hour
#endif

#ifndef MIN_SEGMENT_TIME
#define MIN_SEGMENT_TIME        0 // in minutes
#endif

#ifndef MAX_SEGMENT_TIME
#define MAX_SEGMENT_TIME        24 * 60 // in minutes (1 dag)
#endif

#ifndef SCHEDULE_SAMPLE_TIME
#define SCHEDULE_SAMPLE_TIME    1000 // in ms
#endif

#ifndef CHECK_TEMP_SAMPLE_TIME
#define CHECK_TEMP_SAMPLE_TIME  1000 // in ms
#endif

#ifndef FAN_ON_TEMP
#define FAN_ON_TEMP             50.0 // degrees Celcius
#endif

#ifndef LAMP_ON_TEMP
#define LAMP_ON_TEMP            40.0 // degrees Celcius
#endif

typedef enum {
    SWITCHED_OFF,
    HOLD,
    RAMP
} tempMode_t;

typedef enum {
    ROOTPAGE,
    PREVPAGE,
    NEXTPAGE,
    PREVSELECTPAGE,
    NEXTSELECTPAGE,
    EDITSCHEDULESPAGE,
    ACTIONPAGE,
    SWITCHOVENONPAGE,
    SWITCHOVENOFFPAGE
} webPage_t;

struct schedulePoint_t {
    tempMode_t tempMode = SWITCHED_OFF;
    double tempGoal = MIN_OVEN_TEMP;
    unsigned int timeToNextPoint = MIN_SEGMENT_TIME;
    bool segmentIsUsed = false;
};

struct ovenSchedule_t {
    char scheduleName[64];
    schedulePoint_t schedulePoint[MAX_POINTS_PER_SCHEDULE]; 
};

class PIDController {
public:
	PIDController();

    void begin(int SSRPin, int FANPin, int LAMPPin, int OutputPowerRange = 10000, double initialOvenTemp = -100);
	
	void PIDloop();

    void setControllerOn();

    void setControllerOff();

    void setGoalOvenTemp(double NewOvenTemp, bool ignoreNegativeValue);

    double measureOvenTemps();

    bool getValidTemps();

    bool getTempFault();

    double getInternalTemp();

    double getThermoCoupleTemp();

    bool getAllowPidControllerIsOn();

    bool getSSRIsOn();

    int getSelectedSchedule();

    String getScheduleName();

    bool getScheduleIsEmpty();

    int getCurrentSegment();

    String getCurrentMode();

    double getCurrentGoal();

    unsigned long getTimeLeft();

    void scheduleLoop();

    void addToWebServer(ESP32WebServer &theWebServer, const char *URI, webPage_t webPage);

    void switchOvenOn();

    void switchOvenOff();

    void setUserIsApproved(bool isApproved);

    bool ovenIsSwitchedOn();

    bool ovenIsSwitchedOff();

    void checkTemps();

    void selectSchedule(bool nextSchedule  = false); 

    void handleRootWebPage();

private:
    double currentThermoCoupleTemp;
    double currentInternalTemp;
    bool validTemps = false;

    int SSRPinUsed;
    int FANPinUsed;
    int LAMPPinUsed;
    double setPointTemp;
    double outputPower;
    double currentOvenTemp;

    int SSRPowerRange;

    unsigned long pulseStartTime;
    unsigned long now;

    unsigned long startSampleTime = 0;

    bool allowPIDControllerIsOn = false;
    bool SSRIsOn = false;

    uint8_t fault = 0;
    uint8_t prevFault = 0;
    bool tempFault = false;
    int tempFaultCount = 0;
    bool getNewTemp = true;

    PID * _SSRPID;

    Adafruit_MAX31856 * _thermoCouple;

    ovenSchedule_t allSchedules[MAX_NR_OF_SCHEDULES];
    
    int nrOfSchedules = 0;

    int currentSchedule = 0;

    bool schedulesAreLoaded = false;

    bool scheduleIsLoaded = false;

    int selectedSchedule = 0;

    bool ovenIsOn = false;

    unsigned long ovenStartTime;

    int currentPoint = 0;

    unsigned long currentPointEndTime = 0;

    double previousGoal = 0;

    double nextGoal = 0;

    bool rampIsOn = false;

    double deltaTemp = 0;

    double rampValue = 0;

    unsigned long scheduleStartTime = 0;

    unsigned long segmentStartTime = 0;

    ESP32WebServer * webServer = nullptr;

    bool theUserIsApproved = false;

    bool ovenSwitchedOn = false;

    bool ovenSwitchedOff = false;

    unsigned long checkTempStartTime = 0;

    bool FANIsOn = false;

    bool LAMPIsOn = false;

    void initSchedules();

    bool saveSingleSchedule(int scheduleToSave);

    void loadAllSchedules();
    
    void loadSingleSchedule(int scheduleToLoad);
    
    String formLine_intParam(const char * labelText, const char * paramName, int paramID, int paramValue, int minvalue, int maxvalue, char * unit);

    String formLine_boolParam(const char * labelText, const char * paramName, int paramID, bool paramValue);

    void showSelectedSchedule(String &s);

//    void selectScheduleWebPage();
  
    void editSchedulesWebPage();

    

    void handlePrevWebPage();

    void handleNextWebPage();
    
    void handlePrevSelectWebPage();

    void handleNextSelectWebPage();
    
    void changeParams();

    void handleActionWebPage();

#ifdef DEBUGIT
    void handleSwitchOvenOnPage();

    void handleSwitchOvenOffPage();
#endif

    void selectMode();

   
};
