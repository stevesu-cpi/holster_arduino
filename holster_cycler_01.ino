//code includes counter output to LCD sheild; 11/30/2022 steve su

/**
 * LITH-HSTR Cycle Tester
 * By Stuart Sonatina
 * August 2018
 *
 * This is an arduino script meant to run on an Arduino Nano. Please visit
 * https://www.arduino.cc/en/Main/Software to download the IDE necessary to
 * upload this code to an arduino.
 *
 * This code works with a PD60-3-1161 NEMA23 stepper motor running TMCL code
 * with the following settings. Please download TMCL-IDE at
 * https://www.trinamic.com/support/software/tmcl-ide/ in order to upload code
 * to the stepper driver.
 *
 * MAX_SPEED = 2047       // steps/sec (+/- 0-2047)
 * MAX_ACCEL = 1000       // steps/sec^2 (+/- 0-2047)
 * CUR_LIMIT = 150        // Max current (0-255)

 * SAP 4, 0, MAX_SPEED    // Set max Velocity
 * SAP 5, 0, MAX_ACCEL    // Set max Acceleration
 * SAP 6, 0, CUR_LIMIT    // Set max current
 * SAP 140, 0, 1    // set Microstep Resolution (0 - full, 1 - half, etc)
 * SAP 138, 0, 2    // set Ramp mode (0 - position, 1 - exponential, 2 - velocity)
 * SAP 153, 0, 7    // set Ramp divisor
 * SAP 154, 0, 8    // set Pulse divisor
 * SAP 173, 0, 0    // set stallGuard2 filter enable to standard mode
 */

/**
 * Adjust the following #define numbers as needed to get the desired behavior
 */
#define LOOP_DELAY 1000        // pause time between cycles (ms)
#define FAILURE_ALLOWANCE 5 // Number of times coupler can fail to holster
                            // before program halts

#define CARRIAGE_SPEED 75       // 0-100% (please see TMCL code to adjust maximum)
#define CARRIAGE_TIMEOUT 1500   // How long to allow carriage to run before stopping it (ms)
#define CARRIAGE_OUT_TIME 800   // Time to allow carriage to pull out

#define BUTN_IN 10         // 0-255 button actuator max position
#define BUTN_OUT 200        // 0-255 button actuator min position
#define SAG_UP 100          // 0-255 sag actuator max position
#define SAG_DN 15           // 0-255 sag actuator min position
#define SAG_DN_TIME 1100    // Time to allow sag actuators to move
#define SAG_UP_TIME 1100     // Time to allow sag actuators to move

// Pin definitions
#define CARRIAGE_MOTOR_DIR_PIN 3    // NEMA 23 stepper motor direction (low=forwards)
#define CARRIAGE_MOTOR_ENABLE_PIN 4 // NEMA 23 stepper motor enable
#define CARRIAGE_MOTOR_PIN 5        // NEMA 23 stepper motor PWM control
#define SWITCH_PIN_GND 7  // limit switch ground, not being used currently
#define SWITCH_PIN 8      // limit switch
#define BUTN_PIN 9        // button actuator PWM control
#define SAGL_PIN 10       // left sag actuator PWM control
#define SAGR_PIN 11       // right sag actuator PWM control
#define COUNTER_PIN 12    // outputs to counter
#define NUM_TEST_CYCLES 50000   // number of cycles for test

bool holstered; // flag to see if coupler made it into holster
uint8_t failureCounter = 0;   // Count number of times coupler did not insert properly
uint32_t cycleCounter = 0;    // Count number of cycles, because why not?

/******************************************************************************
 * void setup()
 *****************************************************************************/
void setup()
{
  pinMode(CARRIAGE_MOTOR_DIR_PIN, OUTPUT);
  pinMode(CARRIAGE_MOTOR_ENABLE_PIN, OUTPUT);
  pinMode(CARRIAGE_MOTOR_PIN, OUTPUT);
  pinMode(COUNTER_PIN, OUTPUT);  //output for counter
  digitalWrite(COUNTER_PIN, LOW); //set counter initial state to LOW
  stopMotor();

  pinMode(SWITCH_PIN, INPUT_PULLUP); // INPUT_PULLUP eliminates need for external resistor
  pinMode(SWITCH_PIN_GND, OUTPUT);
  digitalWrite(SWITCH_PIN_GND, LOW); // use adjacent pin as ground

  Serial.begin(9600); // open serial port
  Serial.println("\n----Welcome to LITH-HSTR actuation----");
  while(digitalRead(SWITCH_PIN)==LOW) // limit switch should be wired 'normally closed'
  {
    Serial.println("Please remove coupler from holster, or check that limit switch is plugged in.");
    holstered = true;
    delay(2000);
  }
  holstered = false;

  // initialize actuators
  Serial.println("Initializing motors");
  driveMotor(CARRIAGE_SPEED);
  delay(200);
  driveMotor(-CARRIAGE_SPEED);
  delay(200);
  driveMotor(CARRIAGE_SPEED);
  delay(200);
  stopMotor();
  Serial.println("Motor working");
  delay(2000);

  Serial.println("Button out, sag down");
  analogWrite(BUTN_PIN, BUTN_OUT);
  analogWrite(SAGL_PIN, SAG_DN);
  analogWrite(SAGR_PIN, SAG_DN);
  delay(SAG_DN_TIME);
  Serial.println("Button in, sag up");
  analogWrite(BUTN_PIN, BUTN_IN);
  analogWrite(SAGL_PIN, SAG_UP);
  analogWrite(SAGR_PIN, SAG_UP);
  delay(SAG_UP_TIME);

  delay(2000); // sanity check time! We're about to get started.

  
}

int count=0;
void loop()
{
  if(count == NUM_TEST_CYCLES)stopMotor();
  else
  {
  // Move towards holstered position and wait until it gets there
  Serial.println("Moving to holster");
  driveMotor(CARRIAGE_SPEED);
  uint32_t startTime = millis();  // start timer
  while(!holstered && (millis()-startTime < CARRIAGE_TIMEOUT))
  {
    if(digitalRead(SWITCH_PIN)==LOW) {holstered=true;}
  }
  stopMotor();

  if(holstered) // limit switch has been triggered
  {
    Serial.println("Holstered!");
    failureCounter = 0; // Reset failureCounter
    
    digitalWrite(COUNTER_PIN, HIGH);  //set counter pin to high
    delay(100);
    digitalWrite(COUNTER_PIN, LOW);
    
    Serial.println("Lowering coupler");
    analogWrite(SAGL_PIN, SAG_DN);
    analogWrite(SAGR_PIN, SAG_DN);
    delay(SAG_DN_TIME);

    Serial.println("Lifting coupler and pressing button");
    analogWrite(SAGL_PIN, SAG_UP);
    analogWrite(SAGR_PIN, SAG_UP);
    analogWrite(BUTN_PIN, BUTN_IN);
    delay(SAG_UP_TIME);

    Serial.println("Removing coupler from holster");
    driveMotor(-1*CARRIAGE_SPEED);
    delay(CARRIAGE_OUT_TIME);
    stopMotor();
    analogWrite(BUTN_PIN, BUTN_OUT); // release button

    cycleCounter++;
    Serial.print("********************** Cycle number ");
    Serial.println(cycleCounter);
    Serial.print("********************** Cycle time: ");
    Serial.print(float(millis()-startTime)/1000);
    Serial.println(" seconds");
    //digitalWrite(COUNTER_PIN, LOW);  //set counter pin to low
  }
  else // limit switch was never triggered
  {
    failureCounter++; // Increment counter
    Serial.print("Coupler failed to holster properly ");
    Serial.print(failureCounter);
    Serial.println(" times");

    while(failureCounter >= FAILURE_ALLOWANCE)
    {
      stopMotor();
      Serial.println("\nCoupler failed to holster maximum number of times");
      Serial.println("Please inspect tester and restart arduino");
      delay(10000);
    }

    // remove coupler
    driveMotor(-1*CARRIAGE_SPEED);
    delay(CARRIAGE_OUT_TIME/2);
    stopMotor();

    // shake to try and reset position
    int flip=1;
    for(int i=0; i<3; i++)
    {
      driveMotor(flip*CARRIAGE_SPEED);
      delay(150);
      flip *= -1;
    }
    stopMotor();
  }

  // If coupler is out of holster, set flag to "unholstered"
  if(digitalRead(SWITCH_PIN)==HIGH)
  {
    holstered=false;
  }
  else // Coupler should have been pulled out by now. Stop program.
  {
    do
    {
      Serial.println("Coupler failed to unholster.");
      Serial.println("Remove couple from holster and restart arduino.\n");
      delay(5000);
    } while(true);
  }

  delay(LOOP_DELAY); // in case we want to slow the cycle down

count++;
  }
}

/******************************************************************************
 * Custom functions
 *
 *****************************************************************************/

/******************************************************************************
 * Enable motor and drive at speed from 0-100.
 *****************************************************************************/
void driveMotor(int speed)
{
  if(speed<0)
  {
    digitalWrite(CARRIAGE_MOTOR_DIR_PIN, HIGH);
    speed = -1*speed;
  }
  else
  {
    digitalWrite(CARRIAGE_MOTOR_DIR_PIN, LOW);
  }
  speed = map(speed,0,100,0,255);
  digitalWrite(CARRIAGE_MOTOR_ENABLE_PIN, LOW);   // enable stepper
  analogWrite(CARRIAGE_MOTOR_PIN, speed);          // run motor
}


/******************************************************************************
 * Stop motor by disabling, then drop speed pin to zero.
 *****************************************************************************/
void stopMotor()
{
  digitalWrite(CARRIAGE_MOTOR_ENABLE_PIN, HIGH);   // disable stepper
  analogWrite(CARRIAGE_MOTOR_PIN, 0);             // lower speed to zero
}
