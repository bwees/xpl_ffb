#include <Arduino.h>
#include <SimpleFOC.h>

// Stepper motor instance
StepperMotor motor = StepperMotor(50);
StepperDriver4PWM driver = StepperDriver4PWM(D5, D6, D7, D8);

// encoder instance
MagneticSensorI2C sensor = MagneticSensorI2C(AS5600_I2C);

// commander interface
Commander command = Commander(Serial);

#define REDLINE 170
#define M_VOLTAGE 12
#define F_CONST 1.6

void onMotor(char* cmd){ command.motor(&motor, cmd); }

float airspeed_indicated = 0;
void onAirspeed(char* cmd){ 
  command.scalar(&airspeed_indicated, cmd); 
}

void setup() {

  command.verbose = VerboseMode::on_request;

  // initialize encoder sensor hardware
  sensor.init();
  motor.linkSensor(&sensor);

  // power supply voltage [V]
  driver.voltage_power_supply = 12;
  driver.init();
  // link the motor to the sensor
  motor.linkDriver(&driver);

    // set control loop type to be used
  motor.controller = MotionControlType::torque;
  motor.foc_modulation = FOCModulationType::SinePWM;

  // use monitoring with serial for motor init
  // monitoring port
  Serial.begin(115200);
  // comment out if not needed
  motor.useMonitoring(Serial);

  // initialise motor
  motor.init();
  // align encoder and start FOC 4.39
  motor.initFOC(5.66, Direction::CCW);

  // set the initial target value
  motor.target = 3;

  // define the motor id
  command.add('M', onMotor, "motor");
  command.add('A', onAirspeed, "airspeed");

  Serial.println("READY");
  _delay(1000);
}

float mapfloat(float x, float in_min, float in_max, float out_min, float out_max)
{
 return (x - in_min) * (out_max - out_min) / (in_max - in_min) + out_min;
}


void loop() {
  // iterative setting FOC phase voltage
  motor.loopFOC();

  // iterative function setting the outter loop target
  // velocity, position or voltage
  // if tatget not set in parameter uses motor.target variable
  motor.move();
  motor.monitor();


  // user communication
  command.run();

  float pitch_cmd = mapfloat(motor.shaft_angle, -1.f, -3.f, -1.f, 1.f);

  float relative_force = airspeed_indicated*airspeed_indicated;

  float e_force = pitch_cmd*(relative_force/(REDLINE*REDLINE))*M_VOLTAGE*F_CONST;

 
  motor.target = e_force;
  


}