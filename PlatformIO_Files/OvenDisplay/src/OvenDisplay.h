#pragma once

#include <Arduino.h>
#include <SPI.h>
#include <Adafruit_GFX.h>
#include <Adafruit_ILI9341.h>
#include <XPT2046_Touchscreen.h>
#include "FS.h"

enum messID {
  e_Beat = 1,
  e_Prev,
  e_Next,
  e_On,
  e_Off
};

enum displayMessID {
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

void status(const __FlashStringHelper *msg);

void beginDisplay();

void setupDisplay();

void changeButtons();

void loopDisplay();

void decode_Message_Received(int len);

void read_softw_serial();