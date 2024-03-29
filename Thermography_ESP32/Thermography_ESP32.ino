#include <Wire.h>
#include "MLX90640_API.h"
#include "MLX90640_I2C_Driver.h"
#include "SPI.h"
#include "MLX90640_I2C_Driver.h"
#include "TouchScreen.h"
#include <TFT_eSPI.h>//fast library. SPI Speed can be changed in User_setup.h. currently set at 80MHz which could cause corrupt
// Change to 40Mhz if it happens
#include <driver/dac.h> //Used to drive the DAC and analog out of the ESP32
#include "RunningStat.cpp" //computes the running std with Welford method (1962)
#include <EEPROM.h> //to save the settings
#define EEPROM_SIZE 7 //rawVisualisation, croppingIntegerXM/YM/YP/YM, stdThreshold, stdColorMapping


#define _DEBUG_ //conditional compilation for debug
#define _SERIAL_OUTPUT_

//Functions declaration
void IRAM_ATTR setCalibrationFlag(); //loaded in quick ram
void IRAM_ATTR setRollingAvFlag();
void IRAM_ATTR interruptChangeCroppingMode();
void IRAM_ATTR setRefFrameTakeFlag() ;
void cropDecrease();
void cropIncrease();
void setCroppingInteger();
void setCroppingInteger(int xp, int xm, int yp, int ym);
void setRefFrame();
void getColour(int j);
boolean isConnected();
void drawUI();
void touchScreenHandler();
void serialDoCommand();
void rollingCounterIncrease(int *counter);
void rollingSubstraction(int counter);
void setRollingAverage();
void setTempVisualisation();

float T_max;
float T_min;
float T_center;

//Touchscreen pins
#define YP 26  // must be an analog pin, use "An" notation!
#define XP 27// can be a digital pin
#define YM 32  // can be a digital pin
#define XM 33// must be an analog pin, use "An" notation!
//Touchscreen mapping
#define TS_MINX 150
#define TS_MINY 120
#define TS_MAXX 920
#define TS_MAXY 940
//touchscreen pressure
#define MINPRESSURE 2
#define MAXPRESSURE 5000
TouchScreen ts = TouchScreen(XP, YP, XM, YM, 319); //319ohms resistor across X plate!

TFT_eSPI tft = TFT_eSPI(); //pins are coded in user_setup.h in library
/*
  #define TFT_MOSI 13
  #define TFT_SCLK 14
  #define TFT_CS   15  // Chip select control pin
  #define TFT_DC    2  // Data Command control pin
  #define TFT_RST   18  // Reset pin (could connect to RST pin)

  SCLKmlx90640 22
  SDAmlx90640 21
*/
//Note that other changes have been made in that library.

const byte MLX90640_address = 0x33; //Default 7-bit unshifted address of the MLX90640
paramsMLX90640 mlx90640;
#define TA_SHIFT 8 //Default shift for MLX90640 in open air
static float mlx90640To[768];

int xPos, yPos;
int R_colour, G_colour, B_colour;
int i, j;

bool tempVisualisation = false;
bool rawVisualisation = true;
int incomingByte = 0;
unsigned long frameCounter = 1;
float startingTime;
float currentTime;
float rate;
float timeDelta;

uint16_t mydata[32];
int imageOutput = 0;
int resolutionInteger = 1;
uint16_t reg_value;

float correctionValues[768];

int maxValue = 10;
int minValue = 0;

//Cropping and drawing
int croppingInteger = 0; //The one used with buttons
int croppingIntegerXM = 0; //The ones used from drawing
int croppingIntegerXP = 0;
int croppingIntegerYM = 0;
int croppingIntegerYP = 0;
float rawDataSum = 0;
float rawDataAverage = 0;
int averageCounter = 0;
bool flagDrawingMode;
TSPoint p;

//switch
const int drawingModePin = 23;

//Pushbuttons
const int RollingAvPin = 18;
const int CalibrationPin = 5;
const int refTakePin = 19;
uint32_t pressedTimeStamp;
const uint32_t debounceDelay = 400;
bool flagTakeRefFrame = false;

//Rolling average
int rollingFrame[768];
int rollingFrameMinus[3][768];
int rollingCounter = 0;
bool rollingAverage = false;

float vdd;
float Ta;
bool flagCompareToRefFrame = false;
bool flagRollingAverage = false;
bool flagSetCalibration = false;

//touchscreen
#define BOXSIZE 50
bool flagChangeCroppingMode = false;
bool flagInitPoint = false;
bool flagEndPoint = false;
int initPx = -1; //default is -1 and signals that no point has been selected
int initPy = -1;
int endPx = -1;
int endPy = -1;

//Standard Deviation
RunningStat stdValues[768];
int stdThreshold = 40;
bool stdColorMapping = false;

//histogram
int lut[768];
int hist[256] = {0};
int histCumSum[256] = {0};
float pixelCounter = 0;

//integrate derivatives
int integrateTable[768] = {0};
int lastFrame[768] = {0};
int derivative[768] = {0};
bool doonce = true;

// ***************************************
// **************** SETUP ****************
// ***************************************

void setup()
{
  flagDrawingMode = digitalRead(drawingModePin);
  dac_output_enable(DAC_CHANNEL_1); //enable analog output
  pressedTimeStamp = millis();
  EEPROM.begin(EEPROM_SIZE);
  EEPROMSetupRead();//eeprom reading stored values

  Serial.begin(500000);
  Wire.begin();
  Wire.setClock(400000); //Increase I2C clock speed to 400kHz
  analogReadResolution(10); //required for touchscreen
  pinMode(RollingAvPin, INPUT_PULLUP);
  pinMode(CalibrationPin, INPUT_PULLUP);
  pinMode(refTakePin, INPUT_PULLUP);
  pinMode(drawingModePin, INPUT);
#ifdef _DEBUG_
  pinMode(4, OUTPUT);
#endif
  attachInterrupt(digitalPinToInterrupt(CalibrationPin), setCalibrationFlag, FALLING);
  attachInterrupt(digitalPinToInterrupt(RollingAvPin), setRollingAvFlag, FALLING);
  attachInterrupt(digitalPinToInterrupt(refTakePin), setRefFrameTakeFlag, FALLING);
  attachInterrupt(digitalPinToInterrupt(drawingModePin), interruptChangeCroppingMode, RISING);
  while (!Serial); //Wait for user to open terminal
  Serial.println("MLX90640 IR Array Example");
  if (isConnected() == false)
  {
    Serial.println("MLX90640 not detected at default I2C address. Please check wiring. Freezing.");
    while (1);
  }
  Serial.println("MLX90640 online!");

  //Get device parameters - We only have to do this once
  int status;
  uint16_t eeMLX90640[832];
  status = MLX90640_DumpEE(MLX90640_address, eeMLX90640);
  if (status != 0)
    Serial.println("Failed to load system parameters");
  status = MLX90640_ExtractParameters(eeMLX90640, &mlx90640);
  if (status != 0)
  {
    Serial.println("Parameter extraction failed");
    Serial.print(" status = ");
    Serial.println(status);
  }
  //Once params are extracted, we can release eeMLX90640 array

  //MLX90640_I2CWrite(0x33, 0x800D, 6401);    // writes the value 1901 (HEX) = 6401 (DEC) in the register at position 0x800D to enable reading out the temperatures!!!
  // ===============================================================================================================================================================

  MLX90640_SetRefreshRate(MLX90640_address, 0x04);//required for getFrameData as it is cpu-heavy
  getVddAndTa(&vdd, &Ta, &mlx90640); //uses that frameData to gather Vdd and Ta. TODO check that once in a while ??????

  //MLX90640_SetRefreshRate(MLX90640_address, 0x00); //Set rate to 0.25Hz effective - Works
  //MLX90640_SetRefreshRate(MLX90640_address, 0x01); //Set rate to 0.5Hz effective - Works
  //MLX90640_SetRefreshRate(MLX90640_address, 0x02); //Set rate to 1Hz effective - Works
  //MLX90640_SetRefreshRate(MLX90640_address, 0x03); //Set rate to 2Hz effective - Works
  //MLX90640_SetRefreshRate(MLX90640_address, 0x04); //Set rate to 4Hz effective - Works
  //MLX90640_SetRefreshRate(MLX90640_address, 0x05); //Set rate to 8Hz effective - Works at 800kHz
  //MLX90640_SetRefreshRate(MLX90640_address, 0x06); //Set rate to 16Hz effective - Works at 800kHz
  MLX90640_SetRefreshRate(MLX90640_address, 0x07); //Set rate to 32Hz effective - fails ->OK except temperature reading -> 0x04
  Wire.setClock(1000000); //set i2c speed to 1Mhz, see mlx9060 max speed

  tft.begin();
  tft.setRotation(1);
  if (!flagDrawingMode) {
    drawUI();
  }
  getVddAndTa(&vdd, &Ta, &mlx90640); //required too! why ??
  for (int i = 0; i < 768; i++) {
    correctionValues[i] = (mlx90640.offset[i] * (1 + mlx90640.kta[i] * (Ta - 25)) * (1 + mlx90640.kv[i] * (vdd - 3.3)));
  }
  setRefFrame(); //Gets a starting init frame
  setCalibration();
  tft.fillRect(0, 0, 320, 240, tft.color565(0, 0, 0)); //blackens the screen to reset it
  if (!rawVisualisation) {
    tft.setTextColor(ILI9341_WHITE, tft.color565(0, 0, 0));
    tft.setCursor(0, 60);
    tft.setTextSize(6);
    tft.print("No visualisation mode");
  }
  startingTime = millis();
}



// **********************************
// ************** LOOP **************
// **********************************

void loop()
{
  if (flagDrawingMode) {//handles the touchscreen selection
    touchScreenHandler();
  }

  if (tempVisualisation) {
    temperatureReading();
  }

  else {
    rawReading();
  }

  if (Serial.available() > 0) {
    serialDoCommand();
  }

  //-----------------FLAG HANDLING----------------------------
  if (flagChangeCroppingMode) {
    if (digitalRead(drawingModePin)) {
      setCroppingInteger(0, 320, 0, 240); //Resets to full view
      flagDrawingMode = true;
      tempVisualisation = false;
      MLX90640_SetRefreshRate(MLX90640_address, 0x07);
    }
    else {
      setCroppingInteger(); //gets back to previous cropping integers
      drawUI();
      flagDrawingMode = false;
      tempVisualisation = true;
      MLX90640_SetRefreshRate(MLX90640_address, 0x04);
    }
    flagChangeCroppingMode = false;
  }
  if (flagRollingAverage) {
    setRollingAverage();
    flagRollingAverage = false;
  }
  if (flagSetCalibration) {
    setCalibration();
    flagSetCalibration = false;
  }
  if (flagTakeRefFrame) {
    setRefFrame();
    flagTakeRefFrame = false;
  }
}

//----------------------------------------------------

// ===============================
// =====interruptions handling====
// ===============================
void IRAM_ATTR setRollingAvFlag() {
  flagRollingAverage = true;
}

void IRAM_ATTR setCalibrationFlag() {
  flagSetCalibration = true;
}
void IRAM_ATTR setRefFrameTakeFlag() {
  flagTakeRefFrame = true;
}

void IRAM_ATTR interruptChangeCroppingMode() {
  flagChangeCroppingMode = true;
}


// ===============================
// =====    retrieve data     ====
// ===============================

void getVddAndTa(float *vdd, float *Ta, paramsMLX90640 *mlx90640) {
  static uint16_t mlx90640Frame[834];
  MLX90640_GetFrameData (MLX90640_address, mlx90640Frame);
  *vdd = MLX90640_GetVdd(mlx90640Frame, mlx90640);
  *Ta = MLX90640_GetTa (mlx90640Frame, mlx90640);
}

// ===============================
// =====    rolling average   ====
// ===============================

void rollingCounterIncrease(int *counter) {
  if (*counter < 3) {
    (*counter)++;
  }
  else {
    *counter = 0;
  }
}

void rollingSubstraction(int counter) {
  for (int i = (0 + croppingIntegerYM); i < (24 - croppingIntegerYP); i = i + resolutionInteger) {
    for (int x = (0 + croppingIntegerXM) ; x < (32 - croppingIntegerXP); x = x + resolutionInteger) {
      rollingFrame[32 * i + x] -= rollingFrameMinus[counter][32 * i + x];
    }
  }
}


// ===============================
// =====  Standard Deviation  ====
// ===============================

void clearStdMemory() {
  for (int i = 0; i < 768; i++) {
    stdValues[i].Clear();
  }
  stdThreshold = 25;
}

// ===============================
// =====   SCREEN CROPPING    ====
// ===============================

void cropIncrease() {
  if (millis() > pressedTimeStamp + debounceDelay && croppingInteger <= 10) {
    croppingInteger++;
    setCroppingInteger();
    frameCounter = 0;
    pressedTimeStamp = millis();
    startingTime = pressedTimeStamp;
    tft.fillRect(0, 35, 224, 203, tft.color565(0, 0, 0)); //blackens the screen to reset it
  }
}

void cropDecrease() {
  if (millis() > pressedTimeStamp + debounceDelay && croppingInteger > 0) {
    croppingInteger--;
    setCroppingInteger();
    frameCounter = 0;
    pressedTimeStamp = millis();
    startingTime = pressedTimeStamp;
    tft.fillRect(0, 35, 224, 203, tft.color565(0, 0, 0)); //blackens the screen to reset it
  }
}

void setCroppingInteger() {
  //classic mode with buttons, isotropic cropping
  frameCounter = 0;
  pressedTimeStamp = millis();
  startingTime = pressedTimeStamp;
  tft.fillRect(0, 35, 224, 203, tft.color565(0, 0, 0)); //blackens the screen to reset it
  croppingIntegerXP = croppingInteger;
  croppingIntegerXM = croppingInteger;
  croppingIntegerYP = croppingInteger;
  croppingIntegerYM = croppingInteger;

}

void setCroppingInteger(int xp, int xm, int yp, int ym) { //overloaded functions
  //drawing mode
  frameCounter = 0;
  pressedTimeStamp = millis();
  startingTime = pressedTimeStamp;
  tft.fillRect(0, 0, 320, 240, tft.color565(0, 0, 0)); //blackens the screen to reset it
  if (xm < xp) {
    croppingIntegerXM = xm / 10;
    croppingIntegerXP = 32 - (xp / 10); //because of legacy formula of CroppingInteger with buttons
  }
  else {
    croppingIntegerXM = xp / 10;
    croppingIntegerXP = 32 - (xm / 10);
  }
  if (ym < yp) {
    croppingIntegerYM = ym / 10;
    croppingIntegerYP = 24 - (yp / 10);
  }
  else {
    croppingIntegerYM = yp / 10;
    croppingIntegerYP = 24 - (ym / 10);
  }
  // clears the screen of the window
  initPx = -1;
  initPy = -1;
  endPx = -1;
  endPy = -1;

}


// ===============================
// =====   buttons actions    ====
// ===============================

void setRollingAverage() {
  if (millis() > pressedTimeStamp + debounceDelay) {
    rollingAverage = !rollingAverage;
    frameCounter = 0;
    pressedTimeStamp = millis();
    startingTime = pressedTimeStamp;
    tft.fillRect(0, 35, 224, 203, tft.color565(0, 0, 0)); //blackens the screen to reset it
  }
}

void setTempVisualisation() {
  tempVisualisation = !tempVisualisation;
  if (tempVisualisation == true) {
    MLX90640_SetRefreshRate(MLX90640_address, 0x04);
  }
  else {
    MLX90640_SetRefreshRate(MLX90640_address, 0x07);
  }
}

void setRefFrame() {
  if (millis() > pressedTimeStamp + debounceDelay) {
    for (int i = 0; i < 768; i++) {
      integrateTable[i] = 0;
    }
  }
}

void setCalibration() {
  if (millis() > pressedTimeStamp + debounceDelay) {
    //64 frame calibration
    float value;
    maxValue = 10; //resets basic values
    minValue = 0;

    if (doonce) {
      for (int i = 0; i < 24; i = i + 1) {
        MLX90640_I2CRead(MLX90640_address,  0x0400 + 32 * i,  32, mydata); //read 32 places in memory
        for (int x = 0 ; x < 32; x = x + 1) {
          value = mydata[x];
          if (value > 32767) {
            value = value - 65536;
          }
          else {
            value = value;
          }
          lastFrame[32 * i + x] = value;
          doonce = false;
        }
      }
    }

    for (int w = 0; w < 64; w++) {
      for (int i = (0 + croppingIntegerYM); i < (24 - croppingIntegerYP); i = i + resolutionInteger) {
        MLX90640_I2CRead(MLX90640_address,  0x0400 + 32 * i,  32, mydata); //read 32 places in memory
        for (int x = (0 + croppingIntegerXM) ; x < (32 - croppingIntegerXP); x = x + resolutionInteger) {
          value = mydata[x];
          if (value > 32767) {
            value = value - 65536;
          }
          else {
            value = value;
          }
          derivative[32 * i + x] = value - lastFrame[32 * i + x];
          integrateTable[32 * i + x] += derivative[32 * i + x];
          lastFrame[32 * i + x] = value;
          value = integrateTable[32 * i + x];
          if (value > maxValue && (abs(value) < 1.3 * abs(maxValue) || abs(value) < abs(maxValue) + 10)) {//avoid abberant value
            maxValue = value;
          }
          else if (value < minValue && (abs(value) < 1.3 * abs(minValue) || abs(value) < abs(minValue) + 10)) {
            minValue = value;
          }
        }
      }
    }
    maxValue = (int)(maxValue - 0.15 * abs(maxValue - minValue)); //adjusting values
    minValue = (int)(minValue + 0.15 * abs(maxValue - minValue));

    clearStdMemory();
    calcHistEqualization();
  }
}


// ===============================
// =====      HISTOGRAM       ====
// ===============================

void calcHist() {
  for (int i = 0; i < 256; i++) {
    hist[i] = 0;
  }
  pixelCounter = 0;
  for (int i = (0 + croppingIntegerYM); i < (24 - croppingIntegerYP); i = i + resolutionInteger) {
    MLX90640_I2CRead(MLX90640_address,  0x0400 + 32 * i,  32, mydata); //read 32 places in memory
    for (int x = (0 + croppingIntegerXM) ; x < (32 - croppingIntegerXP); x = x + resolutionInteger) {
      imageOutput = mydata[x];
      if (imageOutput > 32767)
      {
        imageOutput = imageOutput - 65536;
      }
      derivative[32 * i + x] = imageOutput - lastFrame[32 * i + x];
      integrateTable[32 * i + x] += derivative[32 * i + x];
      lastFrame[32 * i + x] = imageOutput;
      imageOutput = integrateTable[32 * i + x];
      imageOutput = constrain(map(imageOutput, minValue, maxValue, 0, 255), 0 , 255);
      hist[imageOutput]++;
      pixelCounter++;
    }
  }
}

void calcHistCumSum() {
  int histSum = 0;
  for (int j = 0; j < 256; j++) {
    histSum += hist[j];
    histCumSum[j] = histSum;
  }
}


void calcHistEqualization() {
  calcHist();
  calcHistCumSum();
  for (int i = 0; i < 256; i++) {
    lut[i] = (int)(histCumSum[i] * (255.00 / pixelCounter));
  }
}



// ===============================
// =====       EEPROM         ====
// ===============================

void EEPROMSetupRead() {
  rawVisualisation = EEPROM.read(0);
  croppingIntegerXM = EEPROM.read(1);
  croppingIntegerYM = EEPROM.read(2);
  croppingIntegerXP = EEPROM.read(3);
  croppingIntegerYP = EEPROM.read(4);
  stdThreshold = EEPROM.read(5);
  stdColorMapping = EEPROM.read(6);
}

void EEPROMCommitValues() {
  EEPROM.write(0, rawVisualisation);
  EEPROM.write(1, croppingIntegerXM);
  EEPROM.write(2, croppingIntegerYM);
  EEPROM.write(3, croppingIntegerXP);
  EEPROM.write(4, croppingIntegerYP);
  EEPROM.write(5, stdThreshold);
  EEPROM.write(6, stdColorMapping);

  EEPROM.commit();
}


// ===============================
// ===== determine the colour ====
// ===============================
/*
   FALSE COLORS
  void getColour(int j)
  {
  if (j >= 0 && j < 30)
  {
    R_colour = 0;
    G_colour = 0;
    B_colour = 20 + (120.0 / 30.0) * j;
  }

  if (j >= 30 && j < 60)
  {
    R_colour = (120.0 / 30) * (j - 30.0);
    G_colour = 0;
    B_colour = 140 - (60.0 / 30.0) * (j - 30.0);
  }

  if (j >= 60 && j < 90)
  {
    R_colour = 120 + (135.0 / 30.0) * (j - 60.0);
    G_colour = 0;
    B_colour = 80 - (70.0 / 30.0) * (j - 60.0);
  }

  if (j >= 90 && j < 120)
  {
    R_colour = 255;
    G_colour = 0 + (60.0 / 30.0) * (j - 90.0);
    B_colour = 10 - (10.0 / 30.0) * (j - 90.0);
  }

  if (j >= 120 && j < 150)
  {
    R_colour = 255;
    G_colour = 60 + (175.0 / 30.0) * (j - 120.0);
    B_colour = 0;
  }

  if (j >= 150 && j <= 180)
  {
    R_colour = 255;
    G_colour = 235 + (20.0 / 30.0) * (j - 150.0);
    B_colour = 0 + 255.0 / 30.0 * (j - 150.0);
  }
  if (j < 0)
  {
    R_colour = 252;
    G_colour = 3;
    B_colour = 169; //purple 252, 3, 169
  }

  }
*/

void getColour(int j) {
  //GRAYSCALE
  if (j < 255 && j > 0) {
    R_colour = j;
    G_colour = j;
    B_colour = j;
  }
  else if (j == -250) { //specific code for green (std dev)
    R_colour = 0;
    G_colour = 250;
    B_colour = 0;
  }
  else if (j < 0) {
    R_colour = 252;
    G_colour = 3;
    B_colour = 169;
  }
  else {
    R_colour = 255;
    G_colour = 255;
    B_colour = 255;
  }
}


void drawUI() {
  //Draws the temp scale, title, T°C integers, ...
  tft.fillScreen(ILI9341_BLACK);
  tft.fillRect(0, 0, 319, 13, tft.color565(255, 0, 10));
  tft.setCursor(100, 3);
  tft.setTextSize(1);
  tft.setTextColor(ILI9341_YELLOW, tft.color565(255, 0, 10));
  tft.print("Thermography");

  tft.drawLine(250, 210 - 0, 258, 210 - 0, tft.color565(255, 255, 255));
  tft.drawLine(250, 210 - 30, 258, 210 - 30, tft.color565(255, 255, 255));
  tft.drawLine(250, 210 - 60, 258, 210 - 60, tft.color565(255, 255, 255));
  tft.drawLine(250, 210 - 90, 258, 210 - 90, tft.color565(255, 255, 255));
  tft.drawLine(250, 210 - 120, 258, 210 - 120, tft.color565(255, 255, 255));
  tft.drawLine(250, 210 - 150, 258, 210 - 150, tft.color565(255, 255, 255));
  tft.drawLine(250, 210 - 180, 258, 210 - 180, tft.color565(255, 255, 255));

  tft.setCursor(80, 220);
  tft.setTextColor(ILI9341_WHITE, tft.color565(0, 0, 0));
  tft.print("T+ = ");


  // drawing the colour-scale
  // ========================

  for (i = 0; i < 181; i++)
  {
    getColour(i);
    tft.drawLine(240, 210 - i, 250, 210 - i, tft.color565(R_colour, G_colour, B_colour));
  }
}

//Returns true if the MLX90640 is detected on the I2C bus
boolean isConnected()
{
  Wire.beginTransmission((uint8_t)MLX90640_address);

  if (Wire.endTransmission() != 0)
    return (false); //Sensor did not ACK

  return (true);
}


void touchScreenHandler() {
  p = ts.getPoint();
  if (p.z > MINPRESSURE && p.z < MAXPRESSURE) {
    // Scale from ~0->1000 to tft.width using the calibration
    TSPoint t; //temporary store value for mapping to screen rotation
    t.x = p.x;
    t.y = p.y;
    p.x = map(t.y, TS_MINY, TS_MAXY, 0, tft.width());
    p.y = map(t.x, TS_MINX, TS_MAXX, tft.height(), 0);

    if (p.x < BOXSIZE && p.y < BOXSIZE) {
      //select init point
      flagInitPoint = true;
      flagEndPoint = false;
    }
    else if (p.x < BOXSIZE && p.y > 240 - BOXSIZE) {
      //select end point
      flagEndPoint = true;
      flagInitPoint = false;
    }
    else if (!(initPx == -1 || initPy == -1 || endPx == -1 || endPy == -1) && (p.x < BOXSIZE && p.y > 120 - (BOXSIZE / 2) && p.y < 120 + (BOXSIZE / 2))) {
      //shoud be faster as if any position is not set, it immediatly returns false due to OR and &&
      setCroppingInteger(endPx, initPx, endPy, initPy);
    }
    if (flagInitPoint) {
      if ((p.x < BOXSIZE && p.y < BOXSIZE) || (p.x < BOXSIZE && p.y > 240 - BOXSIZE) || (p.x < BOXSIZE && (p.y < 120 + (BOXSIZE / 2) && p.y > 120 - (BOXSIZE / 2)))) {
        //do nothing
      }
      else {
        initPx = p.x;
        initPy = p.y;
        flagInitPoint = false;
      }
    }
    else if (flagEndPoint) {
      if ((p.x < BOXSIZE && p.y < BOXSIZE) || (p.x < BOXSIZE && p.y > 240 - BOXSIZE) || (p.x < BOXSIZE && (p.y < 120 + (BOXSIZE / 2) && p.y > 120 - (BOXSIZE / 2)))) {
        //do nothing
      }
      else {
        endPx = p.x;
        endPy = p.y;
        flagEndPoint = false;
      }

    }
  }
}

void drawTouchscreenButtons() {
  if (flagInitPoint) {
    tft.fillRect(0, 0, BOXSIZE, BOXSIZE, tft.color565(3, 78, 252));
  }
  else {
    tft.fillRect(0, 0, BOXSIZE, BOXSIZE, tft.color565(0, 255, 0));
  }
  if (flagEndPoint) {
    tft.fillRect(0 , 240 - BOXSIZE, BOXSIZE, BOXSIZE, tft.color565(3, 78, 252));
  }
  else {
    tft.fillRect(0 , 240 - BOXSIZE, BOXSIZE, BOXSIZE, tft.color565(255, 0, 255));
  }
  tft.fillRect(0, 120 - (BOXSIZE / 2), BOXSIZE, BOXSIZE, tft.color565(255, 255, 0));

  if (!(initPx == -1 || initPy == -1)) {//faster than checking != and &&
    tft.fillCircle(initPx, initPy, 10, tft.color565(0, 255, 0));
  }
  if (!(endPx == -1 || endPy == -1 || initPx == -1 || initPx == -1)) {
    tft.fillCircle(endPx, endPy, 10, tft.color565(255, 0, 255)); //draws end point
    tft.drawLine(initPx, initPy, endPx, initPy, tft.color565(255, 255, 0));//draws yellow window
    tft.drawLine(initPx, initPy, initPx, endPy, tft.color565(255, 255, 0));
    tft.drawLine(endPx, initPy, endPx, endPy, tft.color565(255, 255, 0));
    tft.drawLine(initPx, endPy, endPx, endPy, tft.color565(255, 255, 0));
  }
}

//------------------------------------------------------
//TEMPERATURE READING

void temperatureReading() {
  for (byte x = 0 ; x < 2 ; x++) //Read both subpages
  {
    uint16_t mlx90640Frame[834];
    int status = MLX90640_GetFrameData(MLX90640_address, mlx90640Frame);
    if (status < 0)
    {
      Serial.print("GetFrame Error: ");
      Serial.println(status);
    }

    float Ta = MLX90640_GetTa(mlx90640Frame, &mlx90640);

    float tr = Ta - TA_SHIFT; //Reflected temperature based on the sensor ambient temperature
    float emissivity = 0.95;

    MLX90640_CalculateTo(mlx90640Frame, &mlx90640, emissivity, tr, mlx90640To);
  }

  // determine T_min and T_max and eliminate error pixels
  // ====================================================
  //If pixels are showing errors (wrong temp, ...) simple interpolation to set them to a correct value
  //mlx90640To[1 * 32 + 21] = 0.5 * (mlx90640To[1 * 32 + 20] + mlx90640To[1 * 32 + 22]); // eliminate the error-pixels
  //mlx90640To[4 * 32 + 30] = 0.5 * (mlx90640To[4 * 32 + 29] + mlx90640To[4 * 32 + 31]); // eliminate the error-pixels
  T_min = mlx90640To[0];
  T_max = mlx90640To[0];

  for (i = 1; i < 768; i++)
  {
    if ((mlx90640To[i] > -41) && (mlx90640To[i] < 301))
    {
      if (mlx90640To[i] < T_min)
      {
        T_min = mlx90640To[i];
      }

      if (mlx90640To[i] > T_max)
      {
        T_max = mlx90640To[i];
      }
    }
    else if (i > 0)  // temperature out of range
    {
      mlx90640To[i] = mlx90640To[i - 1];
    }
    else
    {
      mlx90640To[i] = mlx90640To[i + 1];
    }
  }
  // determine T_center
  // ==================
  T_center = mlx90640To[11 * 32 + 15];

  // drawing the picture
  // ===================
  for (i = (0 + croppingInteger) ; i < (24 - croppingInteger) ; i = i + resolutionInteger)
  {
    for (j = (0 + croppingInteger); j < (32 - croppingInteger); j = j + resolutionInteger)
    {
      Serial.print(mlx90640To[i * 32 + j]); ///// RESET VDD to correct value!!!!!! WHY ??????
      Serial.print(" ");
      mlx90640To[i * 32 + j] = 180.0 * (mlx90640To[i * 32 + j] - T_min) / (T_max - T_min);

      getColour(mlx90640To[i * 32 + j]);


      tft.fillRect(217 - j * 7, 35 + i * 7, 7, 7, tft.color565(R_colour, G_colour, B_colour));
    }
    Serial.println("");
  }
  temperatureDrawing(T_max, T_min, T_center);
}

void temperatureDrawing(float T_max, float T_min, float T_center) {
  tft.drawLine(217 - 15 * 7 + 3.5 - 5, 11 * 7 + 35 + 3.5, 217 - 15 * 7 + 3.5 + 5, 11 * 7 + 35 + 3.5, tft.color565(255, 255, 255));
  tft.drawLine(217 - 15 * 7 + 3.5, 11 * 7 + 35 + 3.5 - 5, 217 - 15 * 7 + 3.5, 11 * 7 + 35 + 3.5 + 5,  tft.color565(255, 255, 255));

  tft.fillRect(260, 25, 37, 10, tft.color565(0, 0, 0));
  tft.fillRect(260, 205, 37, 10, tft.color565(0, 0, 0));
  tft.fillRect(115, 220, 37, 10, tft.color565(0, 0, 0));

  tft.setTextColor(ILI9341_WHITE, tft.color565(0, 0, 0));
  tft.setCursor(265, 25);
  tft.print(T_max, 1);
  tft.setCursor(265, 205);
  tft.print(T_min, 1);
  tft.setCursor(120, 220);
  tft.print(T_center, 1);

  tft.setCursor(300, 25);
  tft.print("C");
  tft.setCursor(300, 205);
  tft.print("C");
  tft.setCursor(155, 220);
  tft.print("C");
  tft.setCursor(180, 220);
}

//--------------------------------------------------------
//RAW READING OF PIXEL VALUES

void rawReading() {
#ifdef _DEBUG_
  digitalWrite(4, HIGH);
#endif
  MLX90640_I2CRead(MLX90640_address, 0x8000, 1, &reg_value);
  reg_value = (reg_value >> 3) & 1;//assess bit 3 of 0x8000
  while (!reg_value) {
#ifdef _DEBUG_
    //Serial.println("waiting for a new frame");
    digitalWrite(4, LOW);
    digitalWrite(4, HIGH);
#endif
    MLX90640_I2CRead(MLX90640_address, 0x8000, 1, &reg_value);
    reg_value = (reg_value >> 3) & 1;//assess bit 3 of 0x8000
  }
  MLX90640_I2CRead(MLX90640_address, 0x8000, 1, &reg_value);
  reg_value &= ~0x08; //clears bit 3 of 0x8000 to 0
  MLX90640_I2CWrite(MLX90640_address, 0x8000, reg_value); //resets status register to 0

  for (int i = (0 + croppingIntegerYM); i < (24 - croppingIntegerYP); i = i + resolutionInteger) {
    MLX90640_I2CRead(MLX90640_address,  0x0400 + 32 * i,  32, mydata); //read 32 places in memory
    for (int x = (0 + croppingIntegerXM) ; x < (32 - croppingIntegerXP); x = x + resolutionInteger) {
      imageOutput = mydata[x];
#ifdef _SERIAL_OUTPUT_
      Serial.print(imageOutput);
      Serial.print(F(" "));
#endif
      if (imageOutput > 32767)
      {
        imageOutput = imageOutput - 65536;
      }
      derivative[32 * i + x] = imageOutput - lastFrame[32 * i + x];
      integrateTable[32 * i + x] += derivative[32 * i + x];
      lastFrame[32 * i + x] = imageOutput;
      imageOutput = integrateTable[32 * i + x];
      if (rollingAverage) {
        rollingFrameMinus[rollingCounter][32 * i + x] = imageOutput;
        rollingFrame[32 * i + x] += imageOutput;
      }
      if (rawVisualisation) {
        if (rollingAverage) {
          getColour(map(rollingFrame[32 * i + x] >> 2, minValue, maxValue, 0, 255));
          if (stdValues[32 * i + x].StandardDeviation() > stdThreshold && stdColorMapping) {
            getColour(-250);
          }
        }
        else {
          getColour(map(imageOutput, minValue, maxValue, 0, 255));
          if (stdValues[32 * i + x].StandardDeviation() > stdThreshold && stdColorMapping) {
            getColour(-250);
          }
          //getColour(map(stdValues[32*i+x].StandardDeviation(), 0, 150, 0 , 255)); //std map
        }
        tft.fillRect(x * 10, i * 10, 10, 10, tft.color565(R_colour, G_colour, B_colour)); //draws on the fullscreen
      }
      if (rollingAverage) {
        if (stdValues[32 * i + x].StandardDeviation() > stdThreshold) {
          rawDataSum += map(rollingFrame[32 * i + x] >> 2, minValue, maxValue, 0, 255);
          averageCounter++;
        }
        stdValues[32 * i + x].Push(map(rollingFrame[32 * i + x] >> 2, minValue, maxValue, 0, 255));
      }
      else {
        if (stdValues[32 * i + x].StandardDeviation() > stdThreshold) {
          rawDataSum += map(imageOutput, minValue, maxValue, 0, 255);
          averageCounter++;
        }
        stdValues[32 * i + x].Push(map(imageOutput, minValue, maxValue, 0, 255));
      }
      /*
        #ifdef _SERIAL_OUTPUT_
        Serial.print(imageOutput);
        //imageOutput = constrain(imageOutput, minValue, maxValue);
        //Serial.print(lut[(int)map(imageOutput, minValue, maxValue, 0, 255)]);
        Serial.print(F(" "));
        #endif*/
    }
#ifdef _SERIAL_OUTPUT_
    Serial.println();
#endif
  }// do loop thru all 24 lines!
#ifdef _DEBUG_
  currentTime = millis();
  timeDelta = (currentTime - startingTime) / 1000;
  rate = frameCounter / timeDelta;
  Serial.print("------------------------frame counter = "); Serial.print(frameCounter); Serial.print(" ");
  Serial.print("fps = "); Serial.print(rate); Serial.println("----------------------------------");
  Serial.print("max values : "); Serial.print(maxValue); Serial.print(" "); Serial.println(minValue);
  frameCounter++;
#endif
#ifdef _SERIAL_OUTPUT_
  Serial.println(F("@"));
#endif

  rawDataAverage = abs(rawDataSum / averageCounter);
  rawDataAverage = constrain(rawDataAverage, 0 , 255);
  dac_output_voltage(DAC_CHANNEL_1, rawDataAverage);
  rawDataAverage = 0;
  rawDataSum = 0;
  averageCounter = 0;

  if (rollingAverage) {
    //Substraction from the rolling frame
    rollingCounterIncrease(&rollingCounter);
    rollingSubstraction(rollingCounter);
  }
  if (rawVisualisation) {
    //Draw the init and end point every frame to avoid being erased while refreshing
    drawTouchscreenButtons();
  }
#ifdef _DEBUG_
  digitalWrite(4, LOW);
#endif

}

void serialDoCommand() {
  /*Commands: 0 commit actual values to EEPROM to be stored
              1 enable std deviation color mapping
              2 decrease std thresh
              3 increase std thresh
              4 enables temp display
              5 sets an init frame
              6 switches from rolling average to raw viz
              7 increases cropping
              8 decreases cropping
              9 enable raw display
  */
  incomingByte = Serial.read();
  if (incomingByte == '0') {
    EEPROMCommitValues();
  }
  else if (incomingByte == '9') {
    rawVisualisation = !rawVisualisation;
    tft.fillRect(0, 0, 320, 240, tft.color565(0, 0, 0)); //blackens the screen to reset it
    if (!rawVisualisation) {
      tft.setTextColor(ILI9341_WHITE, tft.color565(0, 0, 0));
      tft.setCursor(0, 60);
      tft.setTextSize(6);
      tft.print("No visualisation mode");
    }
    frameCounter = 0;
    startingTime = millis();

  }
  else if (incomingByte == '1') {
    stdColorMapping = !stdColorMapping;
  }
  else if (incomingByte == '2') {
    stdThreshold--;
  }
  else if (incomingByte == '3') {
    stdThreshold++;
  }
  else if (incomingByte == '4') {
    setTempVisualisation();
  }
  else if (incomingByte == '5') {
    setRefFrame();
  }
  else if (incomingByte == '6') {
    setRollingAverage();
  }
  else if (incomingByte == '7') {
    cropIncrease();
  }
  else if (incomingByte == '8') {
    cropDecrease();
  }
}


