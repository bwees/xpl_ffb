#include <Arduino.h>
#include <SimpleFOC.h>
#include <PicoGamepad.h>

PicoGamepad gamepad;

MagneticSensorSPI pitchEncoder = MagneticSensorSPI(20, 14, 0x3FFF);
MagneticSensorSPI rollEncoder = MagneticSensorSPI(21, 14, 0x3FFF);
 
StepperMotor pitchMotor = StepperMotor(50, 1.5);
StepperDriver4PWM pitchDriver = StepperDriver4PWM(10, 11, 12, 13, 26);

//Mux control pins
#define MUX_S0 4
#define MUX_S1 3
#define MUX_S2 2
#define MUX_S3 7

//Mux in "SIG" pin
#define TOP_MUX_SIG 5
#define BOT_MUX_SIG 6

void setMux(int pin);
void joystick_update();
void init_pitch();
void init_roll();

float targetPitch = 0;

void setup() {
  // monitoring port
  Serial.begin(115200);

  // delay(3000);
  SimpleFOCDebug::enable(&Serial);
  pitchMotor.P_angle.P = 200;

  init_pitch();
  init_roll();




  Serial.println("Encoder ready");

  pinMode(MUX_S0, OUTPUT); 
  pinMode(MUX_S1, OUTPUT); 
  pinMode(MUX_S2, OUTPUT); 
  pinMode(MUX_S3, OUTPUT); 

  digitalWrite(MUX_S0, LOW);
  digitalWrite(MUX_S1, LOW);
  digitalWrite(MUX_S2, LOW);
  digitalWrite(MUX_S3, LOW);

  pinMode(TOP_MUX_SIG, INPUT_PULLUP);
  pinMode(BOT_MUX_SIG, INPUT_PULLUP);
}

void loop() {

  
  pitchEncoder.update();
  rollEncoder.update();

  // // simple P only position control loop
  // float error = targetPitch-pitchEncoder.getAngle()*100;
  // pitchMotor.target = constrain(error*0.18, -15.f, 15.f);
  pitchMotor.target = 3;

  pitchMotor.loopFOC();
  pitchMotor.move();

  Serial.println(pitchMotor.target);

  // joystick_update();

}

void init_pitch() {
  pitchMotor.useMonitoring(Serial);

    // initialise encoder hardware
  pitchEncoder.init();
  rollEncoder.init();

  pitchMotor.linkSensor(&pitchEncoder);
  pitchMotor.foc_modulation = FOCModulationType::SinePWM;

  // power supply voltage [V]
  pitchDriver.init();
  // link the motor to the sensor
  pitchMotor.linkDriver(&pitchDriver);

  pitchMotor.controller = MotionControlType::torque;

   // initialise motor
  pitchMotor.init();
  pitchDriver.voltage_power_supply = 24;
  pitchDriver.pwm_frequency = 20000;
  pitchMotor.voltage_limit = 6;
  pitchMotor.current_limit = 6;

  pitchMotor.zero_electric_angle = .6;
  pitchMotor.sensor_direction = Direction::CW;

  // align encoder and start FOC
  pitchMotor.initFOC();

}

void init_roll() {}

void joystick_update() {
  static int buttons[32];
  static int rmin, rmax, pmin, pmax;

  // display the angle and the angular velocity to the terminal
  float p = pitchEncoder.getAngle()*100;
  float r = rollEncoder.getAngle()*100;

  rmin = min(r, rmin);
  rmax = max(r, rmax);
  pmin = min(p, pmin);
  pmax = max(p, pmax);
  //Loop through and read all 16 values
  //Reports back Value at channel 6 is: 346
  for(int i = 0; i < 16; i ++){
    setMux(i);
    buttons[i] = digitalRead(TOP_MUX_SIG);
    buttons[i+16] = digitalRead(BOT_MUX_SIG);

  }

  for(int i = 0; i < 32; i++)
  {
    // Serial.print(!buttons[i]);
    gamepad.SetButton(i, !buttons[i]);
  }
  // Serial.println();

  gamepad.SetY(map(p, pmax, pmin, -32767, 32767));
  gamepad.SetX(map(r, rmin, rmax, -32767, 32767));


  gamepad.send_update();
}

void setMux(int channel){
  int controlPin[] = {MUX_S0, MUX_S1, MUX_S2, MUX_S3};

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
