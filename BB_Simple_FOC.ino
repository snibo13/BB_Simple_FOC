// Open loop motor control example
#include <SimpleFOC.h>
#include <Wire.h>  // For I2C

#define POLE_PAIRS 15
// BLDC motor & driver instance
BLDCMotor motor = BLDCMotor(POLE_PAIRS);
BLDCDriver6PWM driver = BLDCDriver6PWM(A_PHASE_UH, A_PHASE_UL, A_PHASE_VH, A_PHASE_VL, A_PHASE_WH, A_PHASE_WL);
// Gain calculation shown at https://community.simplefoc.com/t/b-g431b-esc1-current-control/521/21
LowsideCurrentSense currentSense = LowsideCurrentSense(0.003f, -64.0f/7.0f, A_OP1_OUT, A_OP2_OUT, A_OP3_OUT);
HallSensor sensor = HallSensor(A_HALL1, A_HALL2, A_HALL3, POLE_PAIRS);

void doA(){sensor.handleA();}
void doB(){sensor.handleB();}
void doC(){sensor.handleC();}

//target variable
float target_velocity = 0;

// instantiate the commander
Commander command = Commander(Serial);
void doTarget(char* cmd) { command.scalar(&target_velocity, cmd); }
void doMotor(char* cmd) { command.motor(&motor, cmd); }

void setup() {

  
  sensor.init();
  // hardware interrupt enable
  sensor.enableInterrupts(doA, doB, doC);
  motor.linkSensor(&sensor);

  // Driver config
  driver.voltage_power_supply = 22;
  driver.voltage_limit = 1;
  driver.init();
  // link current sense and the driver
  currentSense.linkDriver(&driver);

  // current sensing
  currentSense.init();
  // currentSense.skip_align = true;
  motor.linkCurrentSense(&currentSense);

  motor.linkDriver(&driver);
  motor.voltage_limit = 0.5;   // [V]
 
  // open loop control config
  motor.controller = MotionControlType::velocity;
  // motor.torque_controller = TorqueControlType::voltage;
  // motor.foc_modulation = FOCModulationType::SpaceVectorPWM;

  sensor.init();
  sensor.enableInterrupts(doA, doB, doC);
  motor.linkSensor(&sensor);

  motor.init();
  motor.initFOC();

  command.add('M',doMotor,"motor");
  command.add('T', doTarget, "target velocity");

  Serial.begin(115200);
  motor.useMonitoring(Serial);

  

  // motor.monitor_variables = _MON_TARGET | _MON_VEL | _MON_ANGLE;
  motor.monitor_downsample = 100;
  Serial.println("Motor ready!");
  Serial.println("Set target velocity [rad/s]");
  _delay(1000);
}

void loop() {
  motor.loopFOC();
  motor.move(target_velocity);
  motor.monitor();
  command.run();
}