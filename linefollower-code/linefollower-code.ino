/**********************************************************************************************
   Linefollower - Version 2.2
   by Oleg Samovarov, 4a4ik

   This code is licensed under the MIT License
 **********************************************************************************************/
//  COM порт для bluetooth соединения, используется исходящий!
#include "PID.h"
#include "SimpleTimer.h"  //https://github.com/infomaniac50/SimpleTimer
#include <Pin.h>
#include <EEPROMex.h>


// Variables for PID regulator
double Output = 0, Setpoint = 0, Input = 0;
//On 16MHz micros() has a resolution of four microseconds
int sampleTime = 2000; // in us
// PID coefficient
double Kp = 0.06, Kd = 0.11, Ki = 0.;
//Specify the links and initial tuning parameters
PID myPID(&Input, &Output, &Setpoint, Kp, Ki, Kd, DIRECT);
// nominal and controller maximum output (== maximum speed)
int nom_speed = 50; int max_output = 100;

// Digital sensor array
Pin sens[] = {4, 3, 2, A5, A4, A3, A2, A1, A0, 13, 11, 8, 7, 12, 6, 5};
int myPinsSize = sizeof(sens) / sizeof(Pin); // Store the length of the array of Pins
Pin button = Pin(A7); Pin battery = Pin(A6);
//
int speedM1, speedM2;
bool start = false;
// Variables for loop time check
bool loopTimeCheck = false, debugEnable = false;
unsigned long lastCheck, check;
// Variables for serial interaction, using RoboRemoSPP
char cmd[100];
byte cmdIndex;
// Varianbles for SimpleTimer
SimpleTimer timer;
int timerN[5];


void setup() {
  motorInit();
  Serial.begin(115200);

  for (int i = 0; i < myPinsSize; i++) sens[i].setInputPullupOn();
  button.setInput(); battery.setInput();

  //turn the PID on
  myPID.SetOutputLimits(-max_output, max_output);
  myPID.SetMode(AUTOMATIC);
  //load settings from EEPROM
  loadSettings();
}

void loop() {
  // timer for printing debug info
  if (debugEnable || loopTimeCheck) timer.run();
  // checking loop time
  if (loopTimeCheck) check = micros();


  // ошибка равна 0, когда оба центральных датчика на линии
  Input = (double)linePosition(sens);
  myPID.Compute();
  speedM1 = nom_speed - Output; speedM2 = nom_speed + Output;
  if (start) motorSpeed(speedM1, speedM2);


  /*  check if data has been sent from the computer: */
  while (Serial.available()) {
    /* read the most recent byte */
    char c = Serial.read();
    if (c == '\n') {
      cmd[cmdIndex] = 0;
      exeCmd();  // execute the command
      cmdIndex = 0; // reset the cmdIndex
    } else {
      cmd[cmdIndex] = c;
      if (cmdIndex < 99) cmdIndex++;
    }
  }


  if (loopTimeCheck) lastCheck = micros() - check;

} // LOOP END

void exeCmd() {

  switch (cmd[0]) {
    case 'h': //disable robot
      start = false;
      motorSpeed(0, 0);
      okLog();
      break;
    case 'g': //start robot
      start = true;
      timer.disable(timerN[0]);
      timer.disable(timerN[1]);
      timer.disable(timerN[2]);
      timer.disable(timerN[3]);
      timer.disable(timerN[4]);
      debugEnable = false;
      loopTimeCheck = false;
      okLog();
      break;
    case 'n': //set nominal speed
      nom_speed = atoi(cmd + 1);
      okLog();
      break;
    case 'm': //set maximum speed and output
      max_output = atoi(cmd + 1);
      myPID.SetOutputLimits(-max_output, max_output);
      okLog();
      break;
    case 'p': //set Kp
      Kp = atof(cmd + 1);
      myPID.SetTunings(Kp, Ki, Kd);
      okLog();
      break;
    case 'd': //set Kd
      Kd = atof(cmd + 1);
      myPID.SetTunings(Kp, Ki, Kd);
      okLog();
      break;
    case 'i': //set Ki
      Ki = atof(cmd + 1);
      myPID.SetTunings(Kp, Ki, Kd);
      okLog();
      break;
    case 'w': //go forward
      motorSpeed(50, 50);
      okLog();
      break;
    case 'r': //go right
      motorSpeed(50, 0);
      okLog();
      break;
    case 'a': //go left
      motorSpeed(0, 50);
      okLog();
      break;
    case 's': //save settings to EEPROM
      saveSettings();
      okLog();
      break;
    case 'l': //load settings from EEPROM
      loadSettings();
      okLog();
      break;
    case 'o': //enable debug output
      debugEnable = true;
      timerN[0] = timer.setInterval(500, heartBeat);
      timerN[1] = timer.setInterval(700, batteryVoltage);
      timerN[4] = timer.setInterval(10, errorDifference);
      okLog();
      break;
    case 'f': //disable debug output
      debugEnable = false;
      timer.disable(timerN[0]);
      timer.disable(timerN[1]);
      okLog();
      break;
    case 'z': //enable loop time check
      loopTimeCheck = true;
      timerN[3] = timer.setInterval(300, loopTimeDisplay);
      timerN[2] = timer.setInterval(300, lineOutput);
      okLog();
      break;
    case 'y': //disable loop time check
      loopTimeCheck = false;
      timer.disable(timerN[3]);
      timer.disable(timerN[2]);
      okLog();
      break;

    default:
      Serial.print("* UNK\n");

  } //SWITCH END

} //exeCmd END

void batteryVoltage() {
  String str;
  float v = (float)average(battery) * (4.989 / 1024) * 3.;
  str = "b " + String(v, 2) + '\n';
  Serial.print(str);
}

void heartBeat() {
  Serial.print("x 1\n");
}

void lineOutput() {
  String line = "l ", out;
  for (int i = 0; i < myPinsSize; i++) {
    if (sens[i].getValue()) line += '1';
    else line += '0';
  }
  line += '\n';
  Serial.print(line);

  out = "o Input = " + String(Input, 2) + "  Output " + String(Output, 2) + '\n';
  Serial.print(out);
}

void loopTimeDisplay() {
  String str;
  str = "j " + String(lastCheck) + '\n';
  Serial.print(str);
}

void saveSettings() {
  int adr = 0;

  EEPROM.updateDouble(adr, Kp);
  adr += sizeof(double);
  EEPROM.updateDouble(adr, Kd);
  adr += sizeof(double);
  EEPROM.updateDouble(adr, Ki);
  adr += sizeof(double);
  EEPROM.updateInt(adr, sampleTime);
  adr += sizeof(int);
  EEPROM.updateInt(adr, nom_speed);
  adr += sizeof(int);
  EEPROM.updateInt(adr, max_output);
}

void loadSettings() {
  int adr = 0;

  Kp = EEPROM.readDouble(adr);
  adr += sizeof(double);
  Kd = EEPROM.readDouble(adr);
  adr += sizeof(double);
  Ki = EEPROM.readDouble(adr);
  adr += sizeof(double);
  sampleTime = EEPROM.readInt(adr);
  adr += sizeof(int);
  nom_speed = EEPROM.readInt(adr);
  adr += sizeof(int);
  max_output = EEPROM.readInt(adr);

  myPID.SetTunings(Kp, Ki, Kd);
  myPID.SetOutputLimits(-max_output, max_output);

  printSettings();
}

void printSettings() {
  //  char buff[40];
  //  char b[10];
  //  strcpy(buff, "q Kp =  0.0654 ");
  //  dtostrf(Kp, 0, 4, b);
  //  strcat(buff, b);
  //  strcat(buff, " 0.0654\n");

  String str;
  //printout saved settings
  str = "q Kp = " + String(Kp, 4) + "  Kd = " + String(Kd, 4) + "  Ki = " + String(Ki, 6) +
        + "  Sample time = " + String(sampleTime) + "  Nominal speed = " + String(nom_speed) +
        + "  Maximum speed = " + String(max_output) + '\n';
  //set sliders to saved position
  str += "p " + String(Kp, 4) + "\nd " + String(Kd, 4) + "\ni " + String(Ki, 6) +
         + "\nt " + String(sampleTime) + "\nn " + String(nom_speed) +
         + "\nm " + String(max_output) + '\n';

  Serial.print(str);
  //wait for serial transmission to end
  Serial.flush();
  okLog();
}

void okLog() {
  static char ok[] = "* OK+\n";
  ok[4] = (ok[4] == '-') ? '+' : '-';

  Serial.print(ok);
}

void errorDifference() {
  String str;
  str = "c " + String(myPID.GetErrDiff(), 1) + '\n';
  Serial.print(str);
}
