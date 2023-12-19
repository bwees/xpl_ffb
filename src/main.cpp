#include <Arduino.h>
#include <SimpleFOC.h>
#include <PicoGamepad.h>

PicoGamepad gamepad;

MagneticSensorSPI sensor_pitch = MagneticSensorSPI(20, 14, 0x3FFF);
MagneticSensorSPI sensor_roll = MagneticSensorSPI(21, 14, 0x3FFF);
 
//Mux control pins
int s0 = 4;
int s1 = 3;
int s2 = 2;
int s3 = 7;

//Mux in "SIG" pin
#define TOP_MUX_SIG 5
#define BOT_MUX_SIG 6

void setMux(int pin);

int buttons[32];

int rmin, rmax, pmin, pmax;

void setup() {
  // monitoring port
  Serial.begin(115200);

  // initialise encoder hardware
  sensor_pitch.init();
  sensor_roll.init();

  Serial.println("Encoder ready");

  pinMode(s0, OUTPUT); 
  pinMode(s1, OUTPUT); 
  pinMode(s2, OUTPUT); 
  pinMode(s3, OUTPUT); 

  digitalWrite(s0, LOW);
  digitalWrite(s1, LOW);
  digitalWrite(s2, LOW);
  digitalWrite(s3, LOW);

  pinMode(TOP_MUX_SIG, INPUT_PULLUP);
  pinMode(BOT_MUX_SIG, INPUT_PULLUP);

  Serial.begin(9600);
  sensor_pitch.update();
  sensor_roll.update();
  // display the angle and the angular velocity to the terminal
  float p = sensor_pitch.getAngle()*100;
  float r = sensor_roll.getAngle()*100;
  rmin = r;
  rmax = r;
  pmin = p;
  pmax = p;
}

void loop() {

  
  sensor_pitch.update();
  sensor_roll.update();
  // display the angle and the angular velocity to the terminal
  float p = sensor_pitch.getAngle()*100;
  float r = sensor_roll.getAngle()*100;

  rmin = min(r, rmin);
  rmax = max(r, rmax);
  pmin = min(p, pmin);
  pmax = max(p, pmax);

  Serial.println(r);
  //Loop through and read all 16 values
  //Reports back Value at channel 6 is: 346
  for(int i = 0; i < 16; i ++){
    setMux(i);
    buttons[i] = digitalRead(TOP_MUX_SIG);
    buttons[i+16] = digitalRead(BOT_MUX_SIG);

  }

  for(int i = 0; i < 32; i++)
  {
    gamepad.SetButton(i, !buttons[i]);
  }

  gamepad.SetY(map(p, pmax, pmin, -32767, 32767));
  gamepad.SetX(map(r, rmin, rmax, -32767, 32767));

  gamepad.send_update();

}




void setMux(int channel){
  int controlPin[] = {s0, s1, s2, s3};

  int muxChannel[16][4]={
    {0,0,0,0}, //channel 0
    {1,0,0,0}, //channel 1
    {0,1,0,0}, //channel 2
    {1,1,0,0}, //channel 3
    {0,0,1,0}, //channel 4
    {1,0,1,0}, //channel 5
    {0,1,1,0}, //channel 6
    {1,1,1,0}, //channel 7
    {0,0,0,1}, //channel 8
    {1,0,0,1}, //channel 9
    {0,1,0,1}, //channel 10
    {1,1,0,1}, //channel 11
    {0,0,1,1}, //channel 12
    {1,0,1,1}, //channel 13
    {0,1,1,1}, //channel 14
    {1,1,1,1}  //channel 15
  };

  //loop through the 4 sig
  for(int i = 0; i < 4; i ++){
    digitalWrite(controlPin[i], muxChannel[channel][i]);
  }
  delay(1);
}
