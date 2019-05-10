//Lab6_section2_timer0_starter.ino
#include <Wire.h>
#include <MPU6050.h>

MPU6050 mpu; // declare a variable called mpu of datatype MPU6050
unsigned long timeStampStartOfLoopMs = 0;
float timeStepS = 0.01;
float pitch,roll,yaw = 0.0f; // pitch, roll and yaw values
Vector normalizedGyroDPS; //stores the three gyroscope readings XYZ in degrees per second (DPS)

#define DIM_GREEN 0x00001F
#define DIM_WHITE 0x1F1F1F
#define DIM_GREEN_BLUE 0X001F1F
#define DIM_RED_BLUE 0X1F1F00
#define DIM_RED 0X1F0000
#define DIM_BLUE 0x001F00
#define MED_BLUE 0x005F00
#define DIM_RED_GREEN 0X1F001F
#define GREEN 0x0000FF
#define RED 0xFF0000

#define LED_CLOCK_PIN 3
#define LED_DATA_PIN 2

enum directionState_t {CALIBRATE_MOTOR_STICTION, STOPPED, FORWARD, BACKWARD, LEFT_TURN, RIGHT_TURN};

directionState_t directionState = CALIBRATE_MOTOR_STICTION;
directionState_t previousDirectionState = -1;
int stateTimer = 0;
boolean isNewState = true;
int counter = 0;
int tempYaw = 0;

const int AB_mtr_INA_PIN = 10;
const int AB_mtr_INB_PIN = 9;
const int CD_mtr_INC_PIN = 8;
const int CD_mtr_IND_PIN = 7;

const int PWM_AB_PIN = 6;  
const int PWM_CD_PIN = 5; 

int MotorABpwmOffset = 0;
int MotorCDpwmOffset = 0;

int robot_commanded_heading = 0;

boolean calibrating_motor_stiction = false;


void setup() {
  //Serial.begin(9600);
  Serial.begin(115200);
  // Initialize MPU6050 to have full scale range of 2000 degrees per second
  while(!mpu.begin(MPU6050_SCALE_2000DPS, MPU6050_RANGE_2G)) {
    Serial.println("Could not find a valid MPU6050 sensor, check wiring.");
    delay(1000);
  }
  mpu.calibrateGyro(); // Calibrate gyroscope- must be done with sensor not moving.
  mpu.setThreshold(1);// sets level below which changes in gyro readings are ignored.
                 // helps to reduce noise. 1 = one standard deviation. Range is 0 to 3.

  Serial.println(F("Testing motor A and B using Timer0 in fast PWM mode 3."));
  Serial.println(F("Requires external 9V battery pack.")); 
  
  Serial.print(F("TCCR0A: 0b"));
  Serial.println(TCCR0A, BIN);

  Serial.print(F("TCCR0B: 0b"));
  Serial.println(TCCR0B, BIN);

  Serial.print(F("OCR0A: 0b"));
  Serial.println(OCR0A, BIN);

  Serial.print(F("OCR0B: 0b"));
  Serial.println(OCR0B, BIN);

  Serial.println(F("Configuring Timer0"));
  configureTimer0RegisterForPWMtoDriveMotor();
  Serial.println(F("Timer0 configured"));

  DDRC |= 0x30; //Set PC4 and PC5 to outputs
  
  pinMode(2, OUTPUT);
  pinMode(3, OUTPUT);
  
  
// add your code here, no digitalWrite() or pinMode() commands
}

void configureTimer0RegisterForPWMtoDriveMotor(){
//  TCCR0A |= 0xA3;
//  TCCR0A &= 0xAF;
//  TCCR0B &= 0xF7;
  //Set timer0 to mode 3 (fast PWM) and set COM bits to clear on compare

  OCR0A = 0;
  OCR0B = 0;

  DDRD |= 0xE0;
  DDRB |= 0x07;

  PORTB |= 0x02;
  PORTB &= 0xFB;
  PORTD |= 0x80;
   
  Serial.print(F("TCCR0A: 0b"));
  Serial.println(TCCR0A, BIN);

  Serial.print(F("TCCR0B: 0b"));
  Serial.println(TCCR0B, BIN);

  Serial.print(F("OCR0A: 0b"));
  Serial.println(OCR0A, BIN);

  Serial.print(F("OCR0B: 0b"));
  Serial.println(OCR0B, BIN);

  Serial.print(F("DDRD: 0b"));
  Serial.println(DDRD, BIN);

  Serial.print(F("DDRB: 0b"));
  Serial.println(DDRB, BIN);

  Serial.print(F("PORTD: 0b"));
  Serial.println(PORTD, BIN);

  Serial.print(F("PORTB: 0b"));
  Serial.println(PORTB, BIN);
}

void go_forward(int rate, int steering) {
  digitalWrite(AB_mtr_INA_PIN, 0);
  digitalWrite(AB_mtr_INB_PIN, 1);
  digitalWrite(CD_mtr_INC_PIN, 0);
  digitalWrite(CD_mtr_IND_PIN, 1);
  analogWrite(PWM_AB_PIN, constrain(MotorABpwmOffset + rate - steering, 0, 255));
  analogWrite(PWM_CD_PIN, constrain(MotorCDpwmOffset + rate + steering, 0, 255));
  display_color_on_RGB_led(DIM_GREEN);
}

void go_backward(int rate, int steering) {
  digitalWrite(AB_mtr_INA_PIN, 1);
  digitalWrite(AB_mtr_INB_PIN, 0);
  digitalWrite(CD_mtr_INC_PIN, 1);
  digitalWrite(CD_mtr_IND_PIN, 0);
  analogWrite(PWM_AB_PIN, constrain(MotorABpwmOffset + rate + steering, 0, 255));
  analogWrite(PWM_CD_PIN, constrain(MotorCDpwmOffset + rate - steering, 0, 255));
  display_color_on_RGB_led(DIM_WHITE);
}

void turn_clockwise(int rate) {
  display_color_on_RGB_led(DIM_GREEN_BLUE);
  digitalWrite(AB_mtr_INA_PIN, 0);
  digitalWrite(AB_mtr_INB_PIN, 1);
  digitalWrite(CD_mtr_INC_PIN, 0);
  digitalWrite(CD_mtr_IND_PIN, 0);
  analogWrite(PWM_AB_PIN, constrain(MotorABpwmOffset + rate, 0, 255));
  analogWrite(PWM_CD_PIN, constrain(MotorCDpwmOffset + rate, 0, 255));
}

void turn_counterclockwise(int rate) {
  display_color_on_RGB_led(DIM_RED_BLUE);
  digitalWrite(AB_mtr_INA_PIN, 0);
  digitalWrite(AB_mtr_INB_PIN, 0);
  digitalWrite(CD_mtr_INC_PIN, 0);
  digitalWrite(CD_mtr_IND_PIN, 1);
  analogWrite(PWM_AB_PIN, constrain(MotorABpwmOffset + rate, 0, 255));
  analogWrite(PWM_CD_PIN, constrain(MotorCDpwmOffset + rate, 0, 255));
}

void stop_motor(int rate) {
  PORTD &= 0x00;
  PORTB &= 0xF8;
  display_color_on_RGB_led(DIM_RED);
  OCR0A = rate + MotorABpwmOffset;
  OCR0B = rate + MotorCDpwmOffset;
}

void display_color_on_RGB_led(unsigned long color) {
  unsigned long bitmask=0UL; // UL unsigned long literal (forces compiler to use long data type)
  unsigned long masked_color_result=0UL;
  
  PORTC &= 0xDF; //start with clock low.
  
  for(int i=23; i>=0; i--) { // clock out one data bit at a time, starting with the MSB first
    bitmask= (1UL<<i);    // build bit mask. Note must use "1UL" unsigned long literal, not "1"
    masked_color_result = color & bitmask; // reveals just one bit of color at time
    boolean data_bit=!(masked_color_result==0); // this is the bit of data to be clocked out.
    digitalWrite(LED_DATA_PIN,data_bit);
    
    if(data_bit){ PORTC |= 0x10; }
    else{ PORTC &= 0xEF; }
    
    digitalWrite(LED_CLOCK_PIN,HIGH);
     
    digitalWrite(LED_CLOCK_PIN,LOW);
    
  }
  digitalWrite(LED_CLOCK_PIN,HIGH);  
  delay(1); // after writing data to LED driver, must hold clock line  
            // high for 1 ms to latch color data in led shift register.
}//display_color_on_RGB_led()
//================================================
int find_motorstiction_using_gyro(int fwd_pin, int rev_pin, int pwm_pin){
  calibrating_motor_stiction = true;

  int pwm_value_sent_to_motor = 0;
  
  PORTB |= 0x02;
  PORTB &= 0xFB;
  PORTD |= 0x80;
  
  while(calibrating_motor_stiction){
    normalizedGyroDPS = mpu.readNormalizeGyro();
    analogWrite(pwm_pin, pwm_value_sent_to_motor++);
    
    //delay(5);
    
    if(abs(normalizedGyroDPS.ZAxis) > 10){ 
      calibrating_motor_stiction = false;
      analogWrite(pwm_pin, 0);
      //Serial.println(normalizedGyroDPS.ZAxis * timeStepS); 
    }

    if (pwm_value_sent_to_motor > 250) //failed calibration
    { Serial.print(F("Calibration failed. Check if battery is connected/switched on."));
      analogWrite(pwm_pin, 0);
      while (1) {
        display_color_on_RGB_led(DIM_RED);
        delay(100);
        display_color_on_RGB_led(DIM_BLUE);
        delay(100);
      }
    }
  }
  return pwm_value_sent_to_motor;
}

//================================================================

void robot_goes_forward_at_given_yaw_at_speed(float yaw_heading, int rate) {
  go_forward(rate, 3.0 * (yaw_heading - yaw)); // rate, steering amount
}

//=================================================

void robot_turns_to_heading(float heading){
    if((heading - yaw) < 0){
      turn_clockwise(-50);
    }
    else{
      turn_counterclockwise(-50);
    }
    Serial.println(heading - yaw);
    if( abs(heading - yaw) <= 10) {
      return 0;
    }
  }
//================================================================

void robot_goes_backward_at_given_yaw_at_speed(float yaw_heading, int rate) {
  go_backward(rate, 3.0 * (yaw_heading - yaw)); // rate, steering amount
}
//=================================================

void loop() {
  //Serial.println(directionState);
  timeStampStartOfLoopMs = millis(); // mark the time 
  normalizedGyroDPS = mpu.readNormalizeGyro(); // Read normalized values
  // Calculate Pitch, Roll and Yaw
  pitch = pitch + normalizedGyroDPS.YAxis * timeStepS;
  roll = roll + normalizedGyroDPS.XAxis * timeStepS;
  yaw = yaw + normalizedGyroDPS.ZAxis * timeStepS;
  //Serial.print(pitch);
  //Serial.print(" ");
  //Serial.print(roll);
  //Serial.print(" ");
  //Serial.println(yaw);  
  // Wait until a full timeStepS has passed before next reading
  delay((timeStepS*1000) - (millis() - timeStampStartOfLoopMs));

  timeStampStartOfLoopMs = millis();
  
  isNewState = (directionState != previousDirectionState);
  previousDirectionState = directionState;

  switch(directionState){
    case STOPPED:
      if(isNewState){
        //Housekeeping
        stateTimer = 0;
        stop_motor(0);
        //Serial.println("Entered Stopped");
      }
      stateTimer++;
      if(stateTimer >= 150) directionState = FORWARD;
      break;
    case FORWARD:
      //entry housekeeping
      if (isNewState) {
        robot_commanded_heading = yaw;
        Serial.print("new state is FORWARD, \tcommanded heading is "); 
        Serial.println(robot_commanded_heading);
        display_color_on_RGB_led(DIM_GREEN);
        stateTimer = 0;
      }
      //state business
      stateTimer++;
      robot_goes_forward_at_given_yaw_at_speed(robot_commanded_heading, 15);
      //exit housekeeping
      if(stateTimer >= 200)  directionState = RIGHT_TURN;
      break;

      break;
    case BACKWARD:
      if (isNewState) {
        robot_commanded_heading = yaw;
        Serial.print("new state is BACKWARD, \tcommanded heading is "); 
        Serial.println(robot_commanded_heading);
        display_color_on_RGB_led(DIM_GREEN);
        stateTimer = 0;
      }
      //state business
      stateTimer++;
      robot_goes_backward_at_given_yaw_at_speed(robot_commanded_heading, 15);
      //exit housekeeping
      if(stateTimer >= 200)  directionState = LEFT_TURN;
      break;
    case LEFT_TURN:
      //entry housekeeping
      if (isNewState) {
        robot_commanded_heading = yaw + 90;
        Serial.print(F("new state is LEFT_TURN,\tcommanded heading is "));
        Serial.println(robot_commanded_heading);
        display_color_on_RGB_led(RED);
        stateTimer = 0;
      }
      //state business
      robot_turns_to_heading(robot_commanded_heading);
      stateTimer++;
      //exit housekeeping
      if(stateTimer >= 200) directionState = FORWARD;          // if 2 seconds have passed and yaw=90, change directionState to BACKWARD
      break;
    case RIGHT_TURN:
      //entry housekeeping
      if (isNewState) {
        robot_commanded_heading = yaw - 90;
        Serial.print(F("new state is RIGHT_TURN,\tcommanded heading is "));
        Serial.println(robot_commanded_heading);
        display_color_on_RGB_led(GREEN);
        stateTimer = 0;
      }
      //state business
      robot_turns_to_heading(robot_commanded_heading);
      stateTimer++;
      //exit housekeeping
      if(stateTimer >= 200) directionState = BACKWARD;          // if 2 seconds have passed and yaw=90, change directionState to BACKWARD
      break;
     case CALIBRATE_MOTOR_STICTION:
      //entry housekeeping
      if (isNewState) {
        //find_motorstiction_using_Gyro();
        MotorABpwmOffset = find_motorstiction_using_gyro(AB_mtr_INA_PIN, AB_mtr_INB_PIN, PWM_AB_PIN);
        Serial.print("Motor Phase offset is "); Serial.println(MotorABpwmOffset);
        display_color_on_RGB_led(DIM_GREEN);
        delay(2000);
        MotorCDpwmOffset = find_motorstiction_using_gyro(CD_mtr_INC_PIN, CD_mtr_IND_PIN, PWM_CD_PIN);
        Serial.print("Motor Phase offset is "); Serial.println(MotorCDpwmOffset);
        display_color_on_RGB_led(MED_BLUE);
        delay(2000);
        
      }
      //state business

      //exit housekeeping
      
      directionState = STOPPED;
      Serial.println("Exiting Calibration");
      //Serial.println(isNewState);
      break;
      
    default: directionState = STOPPED;
  }
  //delay(10-(millis() - timeStampStartOfLoopMs));

}
