// Work in progress -- bidirectional DC motor as a servo

// simulate a PWM driven motor controlled by potentiometer
// DaveX 2022-02-21 CC BY-SA
// https://github.com/drf5n/drf5nArduino/tree/main/dc_motor_sim_servo
// This simulates a PID+PWM controlled DC motor with the simulated motor
// control pin 9 OC1A PWM read back from the OCR1A register.
//
// This simulation at https://wokwi.com/projects/362627481524244481
// uses these Uno Connections:
//
//   input: A0: 0-5V potentiometer on A0 sets distance 0-100
//   simulated input: 0-1024 analog distance
//   output: digitalPin9: PWM value to motor
//           digitalPin10 : direction to motor
//   Serial output: of PID state and Motor state
//
// Motor simulated by a state space model in class PmMotor
// This sketch aims at Servo control of a DC motor
// position
//
// https://wokwi.com/arduino/projects/323970337959051859 does speed control
//
// Discussions at:
// https://forum.arduino.cc/t/what-is-the-inverse-of-analogwrite-pin-val/960774/20
// https://forum.arduino.cc/t/why-doesnt-analogwrite-protect-its-writes-to-16bit-registers/961470
// https://forum.arduino.cc/t/motor-rpm-measurement-using-optical-encoder-for-pi-control/960163/13?u=davex
// https://forum.arduino.cc/t/controlling-a-linear-actuator-using-a-pid-controller/1117777/15

const byte motorPwmPin = 9; 
const byte motorDirPin = 10;

#include <PID_v1.h> // https://playground.arduino.cc/Code/PIDLibrary/

class PmMotor { // Permanent Magnet Motor Simulation
  public:
    // DC PM motor state space model per
    // https://ctms.engin.umich.edu/CTMS/index.php?example=MotorSpeed&section=SystemModeling
    // DaveX 2023-04-21
    // input vars
    float V = 5; //  voltage (controlled by pwm fraction)
    int pwm_level; // 0-255 pwm controlled voltage
    // state vars
    float theta = 0;    // rotational position (rads)
    float theta_dot = 0; // rad/sec
    float i_amps = 0 ; // coil current amps
    // outputs
    float theta_dotdot = 0; // rad/sec/sec
    float di_dt;
    // extras;
    float MaxRPM = 1000;
    //constants
    float J = 0.001; // inertia kg*m2
    float B = 0.0001 ; // fluid friction N*m*s
    float L = 0.05; // motor inductance H
    float Ke = 0.001; // speed constant V/rad/sec
    float Kt = 1; // torqe constant N*m/A
    float R = 1.0; // motor resistance ohms
    float i_full = 0.01; // no load current at full speed voltage
    // governing eqns:
    // J*theta_dotdot + B * theta_dot = Kt *i
    // L * di/dt +R*i = V -Ke*theta_dot

    // time variables
    unsigned long updateInterval = 50; // ms
    unsigned long lastUpdate = 0;

    float speed_mrev_sec = 0; //theta_dot
    long millirevs = 0; // theta in millirevolutions
    long encoderTicks = 0;
    const long ticksPerRev = 1000;
    float rpm;

    void setKeMaxRPMVR(float rpm) {
      // set Ke from....
      // assume V voltage, for pwm/255 fraction
      // assume di/dt =0
      // theta_dot_max = rpm/60*2*pi // to rad/sec
      // V*255/255 = R*i  +Ke*theta_dot
      //
      Ke = (V - 0.0 * i_full * R) / (rpm / 60 * 2 * PI);
      MaxRPM = rpm;
    }
    void setInertia(float x) {
      J = x;
    }
    void setResistance(float x) {
      R = x;
    }

    void setKtFromIB() {
      // From J*theta_dotdot + B * theta_dot = Kt *i
      // assume theta_dotdot = 0, full speed, i
      const float theta_dot_max = MaxRPM / 60 * 2 * PI ;
      float myKt = B * theta_dot_max / i_full;

      Serial.print(Kt);
      Serial.print("->");
      Serial.print(myKt);
      Serial.println();

    }
    void update(uint16_t pwm, int dir) {
      // pwm 0-255, as from an analogWrite()
      // dir in HIGH,LOW as from a digitalwrite)
      // One also could change direction or drive voltage with motor.V 
      pwm_level = pwm * (dir == HIGH? 1 : -1);
      update();
    }

    void update() {
      unsigned long now = millis();
      static float i_last = 0;
      if (now - lastUpdate >= updateInterval) {
        lastUpdate += updateInterval;

        i_amps = (V * pwm_level / 255.0 - Ke * theta_dot - L * di_dt) / R;
        di_dt = i_amps - i_last;

        theta += theta_dot * updateInterval / 1000.0;
        theta_dotdot = (Kt * i_amps - B * theta_dot) / J;
        theta_dot += theta_dotdot * updateInterval / 1000.0;
        encoderTicks += theta_dot * updateInterval / 1000.0 / 2 / PI * ticksPerRev;

      }
    }
    void printConfig(void) { // print the configuration vars
      Serial.print("Motor Configuration");
      Serial.print(" MaxRPM=");
      Serial.print(MaxRPM);
      Serial.print(" Ke=");
      Serial.print(Ke,4);
      Serial.print("V/rad/sec, Kt=");
      Serial.print(Kt,4);
      Serial.print("N*m/A B=");
      Serial.print(B,4);
      Serial.print("N*m*s, J=");
      Serial.print(J,4);
      Serial.print("kg*m2, L=");
      Serial.print(L,4);
      Serial.print("H V=");
      Serial.print(V,4);
      Serial.print("V pwm_level=");
      Serial.print(pwm_level);
      Serial.println('\n');
    }
    void printState(void) { // Print the state variables
      Serial.print(" pwm=");
      Serial.print(pwm_level);
      Serial.print(" i_amps=");
      Serial.print(i_amps);
      Serial.print("A theta=");
      Serial.print(theta);
      Serial.print("rad theta_dot=");
      Serial.print(theta_dot);
      Serial.print("rad/s theta_dotdot=");
      Serial.print(theta_dotdot);

      Serial.println("rad/s2");
    }

    void printStateRPM(void) { //print the state in rev & RPM form
      Serial.print(" Motor: pwm=");
      Serial.print(pwm_level);
      Serial.print(" i=");
      Serial.print(i_amps);
      Serial.print("A ");
      Serial.print(theta / 2 / PI);
      Serial.print("rev ");
      Serial.print(theta_dot / 2 / PI * 60, 5);
      Serial.print("RPM ");
      Serial.print(theta_dotdot / 2 / PI * 60, 5);
      Serial.print("RPM/s ");

      Serial.println();
    }
};

PmMotor myMotor;

double myRPM = 0.0;
double mySetpointDisp = 0.0;
double myCV = 0.0;

//PID motorPID(&myRPM, &myCV, &mySetpointRPM,1,0.1,0, DIRECT);

double myDisplacement = 0;
PID motorPID(&myDisplacement, &myCV, &mySetpointDisp, 7, 0.1, 5, DIRECT);

void report (void) {
  const int interval = 500;
  static unsigned long last = - interval;
  unsigned long now = millis();
  if (now - last >= interval) {
    last += interval;
    //motor.printState();
    Serial.print("PID: SP: ");
    Serial.print(mySetpointDisp);
    Serial.print("mm MV: ");
    Serial.print(myDisplacement);
    Serial.print("mm CV: ");
    Serial.print(myCV);
    Serial.print(" counts PWM9: ");
    //    Serial.print(OCR1A,DEC);
    Serial.print(analogGetPwm9(), DEC);
    Serial.println();
    myMotor.printStateRPM();
    //motor.printConfig();
  }
}

void setup() {
  // put your setup code here, to run once:
  Serial.begin(115200);
  myMotor.V = 5;
  myMotor.setResistance(1.2);
  myMotor.setKeMaxRPMVR(30);
  myMotor.i_full = 0.1; // Amps at no-load full speed
  myMotor.B = 0.0001e-0; // friction
  myMotor.setInertia(.1);
  myMotor.setKtFromIB();
  myMotor.printConfig();
  myMotor.printState();

  myRPM = myMotor.theta_dot * 60 / 2 / PI;
  mySetpointDisp = analogRead(A0); // 0-1024 rpm
  //turn the PID on
  motorPID.SetSampleTime(10); //us
  motorPID.SetOutputLimits(-254, 255);
  motorPID.SetMode(AUTOMATIC);
  pinMode(motorPwmPin,OUTPUT);
  pinMode(motorDirPin,OUTPUT);
  
  delay(500);
}

int analogGetPwm9() { // PWM 9 on UNO is OCR1A
  // Retrieve PWM value from pin per https://forum.arduino.cc/t/what-is-the-inverse-of-analogwrite-pin-val/960774/20
  // Uno pin 9 is OC1A per
  // Uno https://docs.arduino.cc/hacking/hardware/PinMapping168
  // Mega https://docs.arduino.cc/hacking/hardware/PinMapping2560

  return ((TCCR1A & bit(COM1A1)) ? OCR1A : digitalRead(9) * 255);
}

void loop() {
  unsigned long loopNow = millis();
  static long bresenhamSlope = 0; // phase error in ticks

  mySetpointDisp = (analogRead(A0) -0) / 10.23; // setpoint of mm based on slider


  myDisplacement = myMotor.theta * 60 / 2 / PI; // measure the motor speed
  //myPhase = bresenhamSlope;  // phase error

  motorPID.Compute();                 // PID to calculate CV from SP and MV
  // control the motor
  analogWrite(motorPwmPin, abs(myCV)); // push PID control value out PWM pin 9
  digitalWrite(motorDirPin,myCV>0? HIGH:LOW);

  // Simulate a motor attached to a PWM pin
  myMotor.update(analogGetPwm9(),digitalRead(motorDirPin));
  report();
}

