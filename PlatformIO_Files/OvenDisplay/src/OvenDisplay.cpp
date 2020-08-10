#include "OvenDisplay.h"
#include "settingsMainOven.h"
#include <time.h>

#include <SoftwareSerial.h>

#define TFT_CS D0  //for D1 mini or TFT I2C Connector Shield (V1.1.0 or later)
#define TFT_DC D8  //for D1 mini or TFT I2C Connector Shield (V1.1.0 or later)
#define TFT_RST -1 //for D1 mini or TFT I2C Connector Shield (V1.1.0 or later)
#define TS_CS D3   //for D1 mini or TFT I2C Connector Shield (V1.1.0 or later)

// #define TFT_CS 14  //for D32 Pro
// #define TFT_DC 27  //for D32 Pro
// #define TFT_RST 33 //for D32 Pro
// #define TS_CS  12 //for D32 Pro

Adafruit_ILI9341 tft = Adafruit_ILI9341(TFT_CS, TFT_DC, TFT_RST);
XPT2046_Touchscreen ts(TS_CS);

// * Initiate Software Serial
SoftwareSerial softw_Serial(SOFTW_SERIAL_RX, SOFTW_SERIAL_TX, false); // (RX, TX. inverted)

// This is calibration data for the raw touch data to the screen coordinates
#define TS_MINX 350
#define TS_MINY 350
#define TS_MAXX 3650
#define TS_MAXY 3800

/******************* UI details */
#define BUTTON_X 40
// #define BUTTON_Y 100
#define BUTTON_Y 241
#define BUTTON_W 60
#define BUTTON_H 30
#define BUTTON_SPACING_X 20
#define BUTTON_SPACING_Y 20
#define BUTTON_TEXTSIZE 2

// text box where text goes
#define TEXT_X 5 // 10
#define TEXT_Y 5 // 10
#define TEXT_W 230
#define TEXT_H 50
#define TEXT_TSIZE 3
#define TEXT_TCOLOR ILI9341_YELLOW
#define BACKGROUND_TCOLOR ILI9341_NAVY
// the data we store in the textfield
#define TEXT_LEN 12
char textfield[TEXT_LEN+1] = "";
uint8_t textfield_i=0;

// Position of the status field
#define STATUS_X 10
// #define STATUS_Y 70
#define STATUS_Y 311

// buttons
#define BUTTON_DEBOUNCE_TIME 100 // in ms

// handshake with controller
#define BEAT_COUNT_SAMPLE_TIME 1000 //  in ms
#define HANDSHAKE_LED_X 120
#define HANDSHAKE_LED_Y 290
#define HANDSHAKE_LED_RADIUS 10

// for textLine function
#define TEXTLINE_X 5

#define MAX_CHARS_PER_LINE 19

char buttonlabels[6][5] = {"<==", "Sel.", "==>", "Off", " ", "On" };
uint16_t buttoncolors[6] = { ILI9341_ORANGE, ILI9341_BLUE, ILI9341_ORANGE,
                             ILI9341_RED, ILI9341_DARKGREY, ILI9341_DARKGREEN };
Adafruit_GFX_Button buttons[6];

unsigned long beatCountStartTime = 0;

bool buttonIsEnabled[6] = { false, false, false, false, false, false };

bool buttonDebounce = false;
unsigned long buttonDebounceStartTime = 0;

// * Set to store received message
char messageReceived[SOFTW_MAXLINELENGTH];
char s2[SOFTW_MAXLINELENGTH];


int currentSchedule = 0;
int currentSegment = 0;
int currentGoal = 0;
unsigned long timeLeft = 0;
int currentOvenTemp = 0;
int statusID = 0;

String scheduleName;

bool ledConnection = false;

unsigned long testSendStartTime = 0;
int testItem = 0;

// Print something in the mini status bar with either flashstring
void status(const __FlashStringHelper *msg) {
  tft.fillRect(STATUS_X, STATUS_Y, 240 - (2 * STATUS_X), 8, ILI9341_BLACK);
  tft.setCursor(STATUS_X, STATUS_Y);
  tft.setTextColor(ILI9341_WHITE, ILI9341_BLACK);
  tft.setTextSize(1);
  tft.print(msg);
}

// Print something on the screen
void textLine(int y, const char * txtLine) {
  char tmpStr[MAX_CHARS_PER_LINE + 1];
  int i;

  i = 0;
  while (txtLine[i]) {
    tmpStr[i] = txtLine[i];
    i++;
  }
  while (i < MAX_CHARS_PER_LINE) {
    tmpStr[i] = ' ';
    i++;
  }
  tmpStr[i] = 0;
  tft.setCursor(TEXTLINE_X, y);
  tft.setTextColor(ILI9341_WHITE, BACKGROUND_TCOLOR);
  tft.setTextSize(2);
  tft.print(tmpStr);
}

// Show if oven is on or off
void theOvenIsOn(bool switchedOn) {
  if (switchedOn) {
    tft.fillRect(55, 192, 130, 25, ILI9341_DARKGREY);
  } else {
    tft.fillRect(55, 192, 130, 25, ILI9341_LIGHTGREY);
  }
  tft.drawRect(55, 192, 130, 25, ILI9341_RED);
  tft.setCursor(60, 198);
  tft.setTextSize(2);
  if (switchedOn) {
    tft.setTextColor(ILI9341_WHITE);
    tft.print("OVEN = ON");
  } else {
    tft.setTextColor(ILI9341_BLUE);
    tft.print("OVEN = OFF");
  }
}

// Show if oven is on or off
void showTemp(int currentTemp) {
  sprintf(s2, "Temp %4d %cC", currentTemp, 247);
  tft.setCursor(TEXT_X + 7, TEXT_Y + 14);
  tft.setTextColor(TEXT_TCOLOR, BACKGROUND_TCOLOR);
  tft.setTextSize(TEXT_TSIZE);
  tft.print(s2);
}

// Clear temp if thermocouple fault
void clearTemp() {
  sprintf(s2, "Temp ---- %cC", 247);
  tft.setCursor(TEXT_X + 7, TEXT_Y + 14);
  tft.setTextColor(TEXT_TCOLOR, BACKGROUND_TCOLOR);
  tft.setTextSize(TEXT_TSIZE);
  tft.print(s2);
}



void changeButtons() {
  tft.drawRect(0, BUTTON_Y, 239, 311, BACKGROUND_TCOLOR);
  for (uint8_t row = 0; row < 2; row++) {
    for (uint8_t col = 0; col < 3; col++) {
      if (buttonIsEnabled[col + row * 3]) {
        buttons[col + row * 3].initButton(&tft, BUTTON_X + col * (BUTTON_W + BUTTON_SPACING_X), 
        BUTTON_Y + row * (BUTTON_H + BUTTON_SPACING_Y),    // x, y, w, h, outline, fill, text
        BUTTON_W, BUTTON_H, ILI9341_WHITE, buttoncolors[col + row * 3], ILI9341_WHITE,
        buttonlabels[col + row * 3], BUTTON_TEXTSIZE); 
        buttons[col + row * 3].drawButton();
      }
    }
  }    
}

void beginDisplay() {

  softw_Serial.begin(BAUD_RATE_SOFTWARE_SERIAL);
  setupDisplay();
  // * Start software serial for connection to controller

}

void setupDisplay() {
  tft.begin();
  
  ts.begin();
  ts.setRotation(2);
  
  tft.fillScreen(BACKGROUND_TCOLOR);
  
  buttonIsEnabled[0] = true;
  buttonIsEnabled[1] = false;
  buttonIsEnabled[2] = true;
  buttonIsEnabled[3] = true;
  buttonIsEnabled[4] = false;
  buttonIsEnabled[5] = true;

  changeButtons();

  tft.drawRect(TEXT_X, TEXT_Y, TEXT_W, TEXT_H, ILI9341_WHITE);  

  status(F("Controller not available yet!..."));
}

void loopDisplay() {
  char messageID[10];
  if (!buttonDebounce) {
    if ((millis() - buttonDebounceStartTime) > BUTTON_DEBOUNCE_TIME) {
      buttonDebounceStartTime = millis();
      TS_Point p;
      bool screenIsTouched = false;

      if (ts.tirqTouched()) {
        if (ts.touched()) {
          p = ts.getPoint();
          screenIsTouched = true;
        }
      }
      if (screenIsTouched) {
        p.x = TS_MAXX - p.x + TS_MINX;
        p.x = map(p.x, TS_MINX, TS_MAXX, 0, tft.width());
        p.y = map(p.y, TS_MINY, TS_MAXY, 0, tft.height());
      }

      // go thru all the buttons, checking if they were pressed
      for (uint8_t b = 0; b < 6; b++) {
        if (buttonIsEnabled[b]) {
          if (screenIsTouched && buttons[b].contains(p.x, p.y)) {
            buttons[b].press(true);  // tell the button it is pressed
          } else {
            buttons[b].press(false);  // tell the button it is NOT pressed
          }
        }
      }

      // now we can ask the buttons if their state has changed
      for (uint8_t b = 0; b < 6; b++) {
        if (buttonIsEnabled[b]) {
          if (buttons[b].justReleased()) {
            buttons[b].drawButton();  // draw normal
          }
          
          if (buttons[b].justPressed()) {
            buttons[b].drawButton(true);  // draw invert!

            switch (b) {
            case 0:
              // Left button pressed
              sprintf(messageID, "%d", e_Prev);
              softw_Serial.println(messageID);
              break;
            case 1:
              // not used
              break;
            case 2:
              // Right button pressed
              sprintf(messageID, "%d", e_Next);
              softw_Serial.println(messageID);
              break;
            case 3:
              // Oven off button pressed
              sprintf(messageID, "%d", e_Off);
              softw_Serial.println(messageID);
              break;
            case 4:
              // not used
              break;
            case 5:
              // Oven on button pressed
              sprintf(messageID, "%d", e_On);
              softw_Serial.println(messageID);
              break;
            
            default:
              break;
            }

            yield();
          }
        }
      }
    }
  }
  if ((millis() - beatCountStartTime) > BEAT_COUNT_SAMPLE_TIME) {
    beatCountStartTime = millis();
    sprintf(messageID, "%d", e_Beat);
    softw_Serial.println(messageID);
#ifdef DEBUGIT    
    Serial.println(F("Message sent: BEAT"));
#endif
  }
}

void showLed(bool & ledIsOn) {
  if (ledIsOn) {
    tft.fillCircle(HANDSHAKE_LED_X, HANDSHAKE_LED_Y, HANDSHAKE_LED_RADIUS, ILI9341_RED);
  } else {
    tft.fillCircle(HANDSHAKE_LED_X, HANDSHAKE_LED_Y, HANDSHAKE_LED_RADIUS, BACKGROUND_TCOLOR);
  }
  ledIsOn = !ledIsOn;
}

void getValue(int len) {
  int i;
  int j;
  
  i = 0;
  s2[0] = 0;
  while ((messageReceived[i] != ':') && (i < len)) {
    i++;
  }
  i++;
  j = 0;
  while (messageReceived[i] && (i < len)) {
    s2[j] = messageReceived[i];
    i++;
    j++;
  }
  s2[j] = 0;
}  

void decode_Message_Received(int len)
{
  String tmpStr;
  char resultStr[SOFTW_MAXLINELENGTH];
  int displayMessageID = 0;
  time_t t;
  tm *local_Time;

  getValue(len);
  if (((String)messageReceived).indexOf(":") > 0) {
    messageReceived[((String)messageReceived).indexOf(":")] = 0;
  }

  displayMessageID = atoi(messageReceived);

  switch (displayMessageID) {
  case e_BeatAnswer:
    showLed(ledConnection);
    break;    

  case e_CurrentSchedule:
    currentSchedule = ((String)s2).toInt();
    sprintf(resultStr, "Schedule: %d", currentSchedule + 1);
    textLine(70, resultStr);
    break;

  case e_ScheduleName:
    sprintf(resultStr, "Name: %s", s2);
    resultStr[26] = 0;
    textLine(90, resultStr);
    break;

  case e_CurrentSegment:
    currentSegment = ((String)s2).toInt();
    sprintf(resultStr, "Segment: %d", currentSegment + 1);
    textLine(110, resultStr);
    break;

  case e_Mode:
    sprintf(resultStr, "Mode: %s", s2);
    resultStr[26] = 0;
    textLine(130, resultStr);
    break;

  case e_Goal:
    currentGoal = ((String)s2).toInt();
    sprintf(resultStr, "Goal: %d %cC", currentGoal, 247);
    textLine(150, resultStr);
    break;

  case e_TimeLeft:          
    t = atoi(s2);
    local_Time = localtime(&t);
    sprintf(resultStr, "Left: %02d:%02d:%02d h", local_Time->tm_hour, local_Time->tm_min, local_Time->tm_sec);
    textLine(170, resultStr);
    break;

  case e_Empty:
    textLine(110, "");
    textLine(130, "");
    textLine(150, "");
    textLine(170, "");
    textLine(130, "SCHEDULE IS EMPTY!");
    break;

  case e_IsOn:
    theOvenIsOn(true);
    break;

  case e_IsOff:
    theOvenIsOn(false);
    break;

  case e_Fault:
    status(F("Thermocouple fault, check controller!"));
    textLine(110, "");
    textLine(130, "");
    textLine(150, "");
    textLine(170, "");
    textLine(130, "Thermocouple Fault!");
    clearTemp();
    break;

  case e_Solved:
    status(F(""));
    textLine(110, "");
    textLine(130, "");
    textLine(150, "");
    textLine(170, "");
    break;

  case e_OvenTemp:
    currentOvenTemp = ((String)s2).toInt();
    showTemp(currentOvenTemp);
    break;

  case e_Authenticate:
    textLine(110, "");
    textLine(130, "");
    textLine(150, "");
    textLine(170, "");
    textLine(130, "Please authenticate");
    textLine(150, "with RFID card or");
    textLine(170, "tag first");
    break;

  case e_ClearMessage:
    textLine(110, "");
    textLine(130, "");
    textLine(150, "");
    textLine(170, "");
    break;

  case e_Status:
    statusID = ((String)s2).toInt();
    switch (statusID) {
    case 0:
      status(F(" "));
      break;
    case 1:
      status(F("Please wait, controller is starting"));
      break;
    case 2:
      status(F("User authenticated"));
      break;
    case 3:
      status(F("User denied"));
      break;
    case 4:
      status(F("No authentication"));
      break;
    case 5:
      status(F(" "));
      break;
    }
    break;
  }
}

void read_softw_serial()
{
  if (softw_Serial.available())
  {
//    memset(messageReceived, 0, sizeof(messageReceived));

    while (softw_Serial.available())
    {
      ESP.wdtDisable();
      int len = softw_Serial.readBytesUntil('\n', messageReceived, SOFTW_MAXLINELENGTH - 1);
      ESP.wdtEnable(1);
      messageReceived[len] = 0;
#ifdef DEBUGIT      
      Serial.printf("Message received: %s\n\r", messageReceived);
#endif

      decode_Message_Received(len);

      yield();
    }
  }
}

