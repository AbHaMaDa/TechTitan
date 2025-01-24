#include <Arduino.h>
#include <PID_v1.h>


#define MOTOR_L_IN1       13
#define MOTOR_L_IN2       12
#define MOTOR_R_IN3       11
#define MOTOR_R_IN4       8 
#define speedControlerR   10 
#define speedControlerL   9

#define IR1 2
#define IR2 3
#define IR3 4
#define IR4 5
#define IR5 6

// variables that are used in the code
int speedValue ;
char mode = 'L';   // L || O || P
char order;
int baseSpeed ;
int leftSpeed ;
int rightSpeed;
double lastError = 0 ;

// PID parameters
double Kp = 0.8, Ki = 0.1, Kd = 0.2; // These values are just examples
double setPoint = 0; // Target value (we want the robot to stay on the line)
double input = 0, output = 0; // PID input and output

PID mypid(&input, &output, &setPoint, Kp, Ki, Kd, DIRECT);// Create a PID object

// function prototypes that are used in the code
void LineFollowingMode();
void PickPLaceMode();
void movingForward();
void movingBackward();
void turningRight();  
void turningLeft();
void stopMoving();
void movingForwardLeft();
void movingForwardRight();
void movingBackwardLeft();
void movingBackwardRight();
void speedUp();
void speedDown();

// the setup function runs once when you press reset or power the board
void setup() {





  // set the pins to be output or input
  pinMode(MOTOR_L_IN1, OUTPUT);
  pinMode(MOTOR_L_IN2, OUTPUT);
  pinMode(MOTOR_R_IN3, OUTPUT);
  pinMode(MOTOR_R_IN4, OUTPUT);
  pinMode(speedControlerR, OUTPUT);
  pinMode(speedControlerL, OUTPUT);
  pinMode(IR1, INPUT);
  pinMode(IR2, INPUT);
  pinMode(IR3, INPUT);
  pinMode(IR4, INPUT);
  pinMode(IR5, INPUT);

  mypid.SetMode(AUTOMATIC); // Set the PID to automatic mode
  mypid.SetOutputLimits(-255, 255); // Set the output limits to be between -255 and 255


  Serial.begin(9600);
  Serial.println("TechTitan is Ready!");
  Serial.println(" You Now Are In Line Following Mode");


}

void loop() {
  // check if the serial port is available
    if (Serial.available()) {
      Serial.println("Motor Control Program Initialized");
      char input = Serial.read(); // read the incoming byte:
      Serial.print("Received: ");
      Serial.println(input);

      if(input == 'L' || input == 'O' || input == 'P'){ // check if the input is one of the modes
          mode = input;
          Serial.println(" You Now Are In Line Following Mode");
          switch (mode){
          case 'L':Serial.println(" You Now Are In Line Following Mode");break;
          case 'O':Serial.println(" You Now Are In Obstacle Avoidance Mode");break;
          case 'P':Serial.println(" You Now Are In pick & place Mode");break;
          }
      }else{
        order = input;
        Serial.print("Received: ");
        Serial.println(order);
      }
  }

  switch (mode){
    case 'L':LineFollowingMode();break;
    case 'O':;break;
    case 'P':PickPLaceMode();break;
  }


  



}
// function definitions
void LineFollowingMode(){ //1st mode

  // read the values of the IR sensors
  int IRsValues[5] ={digitalRead(IR1),digitalRead(IR2),digitalRead(IR3),digitalRead(IR4),digitalRead(IR5)};
  bool allWhite =true;
  bool allBlack = true;
  int weight[5] = {-2,-1,0,1,2}; // The weight of each sensor
  int sum = 0 , count = 0;

  for (int i = 0; i < 5; i++){
    if(IRsValues[i] == 1){
      allWhite = false;
      sum += weight[i];
      count++;
    }
  }

  if(count > 0){

      allBlack = count==5?true:false;

    input = sum / (double)count;}
else{
  input = 0;
}

// Stop the robot if all sensors are 1 (all black) or 0 (all white)
if (allBlack || allWhite) {
  stopMoving();
}



mypid.Compute(); // Compute the PID output


baseSpeed = 150 ;
leftSpeed = baseSpeed + output;
rightSpeed = baseSpeed - output;

 // Constrain motor speeds to prevent exceeding max PWM value
  leftSpeed = constrain(leftSpeed, 0, 255);
  rightSpeed = constrain(rightSpeed, 0, 255);



analogWrite(speedControlerL, leftSpeed);
analogWrite(speedControlerR, rightSpeed);




  Serial.print("Sensor Values: ");
  for (int i = 0; i < 5; i++) {
    Serial.print(IRsValues[i]);
    Serial.print(" ");
  }
  Serial.print(" | Error: ");
  Serial.print(input);
  Serial.print(" | PID Output: ");
  Serial.println(output);

  delay(20); // Adjust delay for response time (lower for faster response)

}




void PickPLaceMode(){ //3th mode

  speedValue = 200;
  // set the speed value to be between 0 and 255
  speedValue = constrain(speedValue, 0, 255);
  // check the order and execute the corresponding function
  switch (order){
    case 'W':movingForward();break;
    case 'S':movingBackward();break;
    case 'D':turningRight();break;
    case 'A':turningLeft();break;
    case 'K':stopMoving();break;
    case 'Q':movingForwardLeft();break;
    case 'E':movingForwardRight();break;
    case 'Z':movingBackwardLeft();break;
    case 'C':movingBackwardRight();break;
    case 'U':speedUp();break;
    case 'X':speedDown();break;
  }
  order = ' ';
}

void movingForward(){
  digitalWrite(MOTOR_L_IN1, HIGH);
  digitalWrite(MOTOR_L_IN2, LOW);
  digitalWrite(MOTOR_R_IN3, HIGH);
  digitalWrite(MOTOR_R_IN4, LOW);
  analogWrite(speedControlerR, speedValue);
  analogWrite(speedControlerL, speedValue);
  Serial.println("Moving Forward");

}

void movingBackward(){
  digitalWrite(MOTOR_L_IN1, LOW);
  digitalWrite(MOTOR_L_IN2, HIGH);
  digitalWrite(MOTOR_R_IN3, LOW);
  digitalWrite(MOTOR_R_IN4, HIGH);
  analogWrite(speedControlerR, speedValue);
  analogWrite(speedControlerL, speedValue);
    Serial.println("Moving Backward");

}

void turningRight(){
  digitalWrite(MOTOR_L_IN1, HIGH);
  digitalWrite(MOTOR_L_IN2, LOW);
  digitalWrite(MOTOR_R_IN3, LOW);
  digitalWrite(MOTOR_R_IN4, HIGH);
  analogWrite(speedControlerR, speedValue);
  analogWrite(speedControlerL, speedValue);
  Serial.println("Turning Right");

}

void turningLeft(){
  digitalWrite(MOTOR_L_IN1, LOW);
  digitalWrite(MOTOR_L_IN2, HIGH);
  digitalWrite(MOTOR_R_IN3, HIGH);
  digitalWrite(MOTOR_R_IN4, LOW);
  analogWrite(speedControlerR, speedValue);
  analogWrite(speedControlerL, speedValue);
  Serial.println("Turning Left");

}
void stopMoving(){
  digitalWrite(MOTOR_L_IN1, LOW);
  digitalWrite(MOTOR_L_IN2, LOW);
  digitalWrite(MOTOR_R_IN3, LOW);
  digitalWrite(MOTOR_R_IN4, LOW);
  analogWrite(speedControlerR, 0);
  analogWrite(speedControlerL, 0);
  Serial.println("Stop Moving");
}
void movingForwardLeft(){
  digitalWrite(MOTOR_L_IN1, HIGH);
  digitalWrite(MOTOR_L_IN2, LOW);
  digitalWrite(MOTOR_R_IN3, HIGH);
  digitalWrite(MOTOR_R_IN4, LOW);
  analogWrite(speedControlerR, speedValue);
  analogWrite(speedControlerL, speedValue-40);
  Serial.println("Moving Forward Left");
}
void movingForwardRight(){
  digitalWrite(MOTOR_L_IN1, HIGH);
  digitalWrite(MOTOR_L_IN2, LOW);
  digitalWrite(MOTOR_R_IN3, HIGH);
  digitalWrite(MOTOR_R_IN4, LOW);
  analogWrite(speedControlerR, speedValue-40);
  analogWrite(speedControlerL, speedValue);
  Serial.println("Moving Forward Right");
}
void movingBackwardLeft(){
  digitalWrite(MOTOR_L_IN1, LOW);
  digitalWrite(MOTOR_L_IN2, HIGH);
  digitalWrite(MOTOR_R_IN3, LOW);
  digitalWrite(MOTOR_R_IN4, HIGH);
  analogWrite(speedControlerR, speedValue);
  analogWrite(speedControlerL, speedValue-40);
  Serial.println("Moving Backward Left");
}

void movingBackwardRight(){
  digitalWrite(MOTOR_L_IN1, LOW);
  digitalWrite(MOTOR_L_IN2, HIGH);
  digitalWrite(MOTOR_R_IN3, LOW);
  digitalWrite(MOTOR_R_IN4, HIGH);
  analogWrite(speedControlerR, speedValue-40);
  analogWrite(speedControlerL, speedValue);
  Serial.println("Moving Backward Right");
}
void speedUp(){

  speedValue += 50;
  speedValue = constrain(speedValue, 0, 255);
  analogWrite(speedControlerR, speedValue);
  analogWrite(speedControlerL, speedValue);
    Serial.print("updated speed: ");
    Serial.println(speedValue);
  
}

void speedDown(){
  speedValue -= 50;
  speedValue = constrain(speedValue, 0, 255);
  analogWrite(speedControlerR, speedValue);
  analogWrite(speedControlerL, speedValue);
  Serial.print("updated speed: ");
  Serial.println(speedValue);
}

