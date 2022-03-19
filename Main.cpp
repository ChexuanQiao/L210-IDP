#include <Wire.h>
#include <Arduino.h>
#include <Adafruit_MotorShield.h>
#include "utility/Adafruit_MS_PWMServoDriver.h"
#include <ArduinoQueue.h>
#include <Servo.h>

Adafruit_MotorShield AFMS = Adafruit_MotorShield();
// Motor shield motor pins.
Adafruit_DCMotor *LeftMotor = AFMS.getMotor(3);
Adafruit_DCMotor *RightMotor = AFMS.getMotor(4);

// Arduino Pins
const int ButtonPin = 11;
const int AmberLED = 8;

const int GreenLED = 12;
const int RedLED = 13;

// Line sensor data receive pins.
const int L2LF_receive = 6;// = 12;
const int L1LF_receive = 5;// = 11;
const int R1LF_receive = 7;// = 10;
const int R2LF_receive = 4;// = 8;

const int clamp_servo_pin = 10; //Servo 1 on the outside;
const int chassis_servo_pin = 9; // Servo 2 on the inside;
// ------------------------------

Servo clamp_servo;  // create servo object to control a servo
Servo chassis_servo;

// tuned servo positions.
const int chassis_high = 25;
const int chassis_low = 48;
const int clamp_open = 65;
const int clamp_closed = 105;

// identifiers to record which of the targets are occupied.
int far_blue_target = 0;
int close_blue_target = 0;
int far_red_target = 0;
int close_red_target = 0;

// identifiers to record to which target we are delivering to.
int delivering_far = 0;
int delivering_close = 0;

// Line deviate failsafe
int stray_left;
const int stray_threshold = 6;

// TASK MANAGER
int buttonState = 0;
int button_pressed = 0;
int task = 0;
int block_found = 0;
int block_approached = 0;
int block_color = 0; // 0 for blue, 1 for red.
int block_picked = 0;
int retreated_with_block = 0;
int block_placed = 0;
int block_number = 1;
int journey = 0; // 0 for go to, 1 for return
int Turn = 0;

// Tunable Parameters.
const float kp = 30; // Proportional gain.
const float ki = 0; // Integral gain.
const float kd = 0; // Derivative gain.

const int main_loop_delay_time = 50; // main loop delay.
const int print_freq = 500 / main_loop_delay_time; // Print every 0.5s.
const int max_speed = 255; // Maximum allowable motor speed.
const int ref_speed = 200; // Normal forward motor speed.
const int turn_speed = 200; // Turning speed.

const int intxn_queue_length = 5; // intersection queue length.
const int intxn_detection_threshold = 2; // IntersectionDetection Threshold, the number of 1s in intersection queue.
const int intxn_deb_time = 1000; // Intersection debounce threshold time.
unsigned long l_intxn_deb_prev = 0; // Last Left intersection debounce start time.
unsigned long r_intxn_deb_prev = 0; // Last Right intersection debounce start time.
int l_intxn_deb = 1; // Intersection debounce checker. 1 for debounce finished.
int r_intxn_deb = 1;

// Search queues. Search queue detects sudden drop only.
const int sweep_queue_length = 10; // Distance sensor sweep queue length.
const int front_queue_length = 3;
const int back_queue_length = 3;
const float dip_threshold = 6;
float front_avg = 0;
float back_avg = 0;
ArduinoQueue<int> Q = ArduinoQueue<int>(sweep_queue_length);
ArduinoQueue<int> F = ArduinoQueue<int>(front_queue_length);
ArduinoQueue<int> B = ArduinoQueue<int>(back_queue_length);
float front_front;
float mid_front;
float back_front;

// LineFollow
float PIDError = 0; // PID control feedback
float P, I, D;
float pre_I = 0;
float pre_P = 0;
int speedL;
int speedR;
int prev_speedL;
int prev_speedR;

unsigned long main_loop_counter = 0;
unsigned long current_time = 0;
unsigned long PID_prev_time = 0; 
unsigned long prev_blink_time = 0;

// record intersections encountered.
int left_intxn_counter = 0;
int right_intxn_counter = 0;
int left_white_counter = 0;
int right_white_counter = 0;

// Line sensor data. 0 (black) or 1 (white).
int L2LF_data;
int L1LF_data;
int R1LF_data;
int R2LF_data;

ArduinoQueue<int> L2Queue = ArduinoQueue<int>(intxn_queue_length);
ArduinoQueue<int> R2Queue = ArduinoQueue<int>(intxn_queue_length);

int current_left_intxn;
int current_right_intxn;

const int delivery_lf_period = 1950;
int approach_time; // calculated time to approach and retreat from block.
int line_find_turn_delay = 0;
float color_baseline = 0;

// DistanceIR params.
float sensorVal = 0;
float sensorVolt = 0;
float Vr = 5.0;
float sum = 0;
float k1=6.84424549;
float k2=-0.61625197;
float DS_data; // Distance Sensor data.
float block_distance;

void BlinkRed(){
    digitalWrite(RedLED, LOW);
    delay(10);
    digitalWrite(RedLED, HIGH);
}
void BlinkGreen(){
    digitalWrite(GreenLED, LOW);
    delay(10);
    digitalWrite(GreenLED, HIGH);
}

class LFDetection
{
public:
    void LFDataRead(void); // Read data from line sensors.
    void DSDataRead(void); // Read data from block_distance sensors.
    void EdgeDetection(void); // Detect horizontal white strips.
    void IntersectionDetection(void); // Count the number of intersections encountered.
    void BlockDetection(void);
    void Color(void);
    void ColorBaseline(void);
};

void LFDetection::ColorBaseline(){
    int counter = 10;
    for (int i = 0 ; i < counter ; i++){
        int sensorValue = analogRead(A0);
        int sensorValue2 = analogRead(A1);
        // Convert the analog reading (which goes from 0 - 1023) to a voltage (0 - 5V):
        float voltage = sensorValue * (5.0 / 1023.0);
        float voltage2 = sensorValue2 * (5.0 / 1023.0);
        float difference = voltage - voltage2;
        /* set baseline difference as an average of ten values, recorded before 
        the actual color sensing. */
        color_baseline += difference / counter;
    }
    Serial.println("");
    Serial.println("Color Baseline: " + String(color_baseline));
    
    digitalWrite(GreenLED, LOW);
    delay(100);
    digitalWrite(GreenLED, HIGH);
    delay(100);
    digitalWrite(RedLED, LOW);
    delay(100);
    digitalWrite(RedLED, HIGH);

    delay(2000);
}

void LFDetection::Color(){
    // read the input on analog pin 0:
    int sensorValue = analogRead(A0);
    int sensorValue2 = analogRead(A1);
    // Convert the analog reading (which goes from 0 - 1023) to a voltage (0 - 5V):
    float voltage = sensorValue * (5.0 / 1023.0);
    float voltage2 = sensorValue2 * (5.0 / 1023.0);
    float difference = voltage - voltage2;
    // print out the value you read:
    Serial.println("V1: " + String(voltage));
    Serial.println("V2:" + String(voltage2));
    Serial.println("difference: " + String(difference));

    float color_threshold = 1;

    if (difference - color_baseline < color_threshold) {
      digitalWrite(GreenLED, LOW); // turn Green LED on
      digitalWrite(RedLED, HIGH); // Red LED off
      Serial.println("Color is BLUE");
      block_color = 0;
      Turn = 1; // left
    }
    else if (difference - color_baseline >= color_threshold){
      digitalWrite(GreenLED, HIGH); // Green LED off
      digitalWrite(RedLED, LOW); // Red LED on
      Serial.println("Color is RED");
      block_color = 1;
      Turn = 2;
    }
    delay(5000);
    digitalWrite(GreenLED, HIGH);
    digitalWrite(RedLED, HIGH);
    return;
}

void LFDetection::BlockDetection(){
  // Fill three queues
  // F----- Q ---------- B-----
  while (!F.isFull()){
    DSDataRead();
    F.enqueue( DS_data );
    front_avg += DS_data / front_queue_length;
  }
  while (!Q.isFull()){
    DSDataRead();
    Q.enqueue( DS_data );
  }
  while (!B.isFull()){
    DSDataRead();
    B.enqueue( DS_data );
    back_avg += DS_data / back_queue_length;
  }
  Serial.println("queues full");

    DSDataRead();
    front_front = F.dequeue();
    mid_front = Q.dequeue();
    back_front = B.dequeue();

    F.enqueue(mid_front);
    Q.enqueue(back_front);
    B.enqueue(DS_data);
    
    // Track the moving average of the frontqueue and backqueue
    front_avg -= front_front / front_queue_length;
    front_avg += mid_front / front_queue_length;
    back_avg -= back_front / back_queue_length;
    back_avg += DS_data / back_queue_length;

    Serial.println("Front Back Average: " + String(front_avg) + "  " + String(back_avg));

    // Check for a significant drop in distance between front and back queue.
    if (front_avg - back_avg > dip_threshold){
      block_distance = DS_data;
      Serial.println(" ");
      Serial.println("Block Found. Distance: " + String(block_distance));
      Serial.println(" ");

      block_found = 1;
    }
}

void LFDetection::LFDataRead(){
    L1LF_data = digitalRead(L1LF_receive);
    R1LF_data = digitalRead(R1LF_receive);
    L2LF_data = digitalRead(L2LF_receive);
    R2LF_data = digitalRead(R2LF_receive);
    /*
    if (R2LF_data > L2LF_data){
        stray_left++;
    }
    if (R2LF_data < L2LF_data){
        stray_left--;
    }
    if (R2LF_data == L2LF_data){
        if (stray_left < 0){stray_left++;}
        if (stray_left > 0){stray_left--;}
    }
     stray_left = 0;
     */
    if (main_loop_counter % print_freq == 0){
      Serial.println("Sensor Data: " + String(L2LF_data) + " " + String(L1LF_data)+" " + String(R1LF_data) + " " + String(R2LF_data));
      // Serial.println("Stray_left:" + String(stray_left));
    }


}

void LFDetection::DSDataRead(){
    sum=0;
    for (int i=0; i<100; i++)
    {
        sum=sum+float(analogRead(A2));
    }
    sensorVal=sum/100;
    sensorVolt=sensorVal*Vr/1024;

    DS_data = pow(sensorVolt*(1/k1), 1/k2);

    Serial.println("Distance sensor: " + String(DS_data));
}

void LFDetection::EdgeDetection(){
    int Ldeq = L2Queue.dequeue();
    int Rdeq = R2Queue.dequeue();
    int L2enqueued = 0;
    int R2enqueued = 0;
    if (l_intxn_deb == 0){ // Debounce not finished.
        L2enqueued = 0;
        L2Queue.enqueue(L2enqueued);
    }
    else{
        L2Queue.enqueue(L2LF_data); 
        L2enqueued = L2LF_data;
    }
    if (r_intxn_deb == 0){ // Debounce not finished.
        R2enqueued = 0;
        R2Queue.enqueue(R2enqueued);
    }
    else{
        R2Queue.enqueue(R2LF_data); 
        R2enqueued = R2LF_data;
    }

    if (L2enqueued == 1){
        left_white_counter++;
         Serial.println("L1 ENqueued 1");
    }
    if (Ldeq == 1){
        left_white_counter--;
        // Serial.println("L1 DEqueued 1");
    }
    if (R2enqueued == 1){
        right_white_counter++;
         Serial.println("R1 ENqueued 1");
    }
    if (Rdeq == 1){
        right_white_counter--;
        // Serial.println("R1 DEqueued 1");
    }
}

void LFDetection::IntersectionDetection(){
    if (current_time - l_intxn_deb_prev > intxn_deb_time){
        l_intxn_deb = 1;
    }
    if (current_time - r_intxn_deb_prev > intxn_deb_time){
        r_intxn_deb = 1;
    }
    if (left_white_counter == intxn_detection_threshold) {
        // Debounce after recorded an intersection.
        l_intxn_deb_prev = current_time;
        l_intxn_deb = 0;

        // Record the intersection.
        left_intxn_counter++;
        Serial.println("Left Intersection " + String(left_intxn_counter) + " recorded. ");
        digitalWrite(GreenLED, LOW);
        delay(10);
        digitalWrite(GreenLED, HIGH);

        // Reset Line follower counter.
        left_white_counter = 0;
        for (int i=0;i<intxn_queue_length;i++){
            L2Queue.dequeue();
            L2Queue.enqueue(0);
        }
    }
    if (right_white_counter == intxn_detection_threshold) {
        // Debounce after recorded an intersection.
        r_intxn_deb_prev = current_time;
        r_intxn_deb = 0;

        // Record the intersection.
        right_intxn_counter++;
        Serial.println("Right Intersection " + String(right_intxn_counter) + " recorded. ");
        digitalWrite(RedLED, LOW);
        delay(10);
        digitalWrite(RedLED, HIGH);
        // Reset Line follower counter.
        right_white_counter = 0;
        for (int i=0;i<intxn_queue_length;i++){
            R2Queue.dequeue();
            R2Queue.enqueue(0);
        }
    }
}

// ******************************************

class ServoMove{
public:
    void Pickup();
    void Place();
    int pos;
};
void ServoMove::Pickup(){
    delay(200);
    // Lower grabber.
    for (int i = chassis_high; i < chassis_low; i++){
        chassis_servo.write(i);
        delay(10);
    }

    delay(200);
    // Grab the block.
    for (int i = clamp_open; i < clamp_closed; i++ ){
        clamp_servo.write(i);
        delay(10);
    }
    delay(200);
    // Raise the grabber.
    for (int i = chassis_low; i > chassis_high; i--){
        chassis_servo.write(i);
        delay(10);
    }
    block_picked = 1;
    delay(500);
}
void ServoMove::Place(){

    delay(200);
    for (int i = chassis_high; i < chassis_low; i++){
        chassis_servo.write(i);
        delay(10);
    }

    delay(200);

    for (int i = clamp_closed; i > clamp_open; i-- ){
        clamp_servo.write(i);
        delay(10);
    }
    delay(200);

    for (int i = chassis_low; i > chassis_high; i--){
        chassis_servo.write(i);
        delay(10);
    }
    delay(500);
    block_placed = 1;
    block_number ++;
    return;
}

class MovementControl: public LFDetection, public ServoMove
{
  public:
      void FindTask();

      void DummyMove();

      void PID();
      void LineFollow();

      void Search();
      void Approach();
      void Pickup();
      void Retreat();
      void LineFindTurn();

      void LFDelivery();
      void Reset();

      void Finish();

      void BlinkAmber();

      void Stray();

      void HardTurn();
      void Stop();
      void HardDelivery();
};

void MovementControl::FindTask(){
    // LFDataRead();
    // EdgeDetection();
    // IntersectionDetection();

    /*
    Task 0: Starting move forward
    Task 1: Line follow
    Task 2: Search turn
    Task 3: Approach
    Task 4: Pick up
    Task 5: Retreat
    Task 6: 180 turn
    Task 7: Delivery
    Task 8: Reset
    */

    if (left_intxn_counter == 0 && right_intxn_counter == 0){
        task = 0;
        if (main_loop_counter % print_freq == 0){
        Serial.println("Dummy Move Forward."); }// "Starting Dummy Move Forward.");
    }

    if ( (1 <= right_intxn_counter <= 2 || 1 <= left_intxn_counter <= 2) && block_found == 0){
        task = 1;
        if (main_loop_counter % print_freq == 0){
        Serial.println("Line Following.");}
    }

    if ((left_intxn_counter == 3 || right_intxn_counter == 3) && block_found == 0){
        task = 2;
        //if (main_loop_counter % print_freq == 0){
        Serial.println("Searching.");}
    //}

    if (block_found == 1 && block_approached == 0
        && block_picked == 0 && retreated_with_block == 0){
        task = 3;

        Serial.println("Approaching block.");
    }

    if (block_found == 1 && block_approached == 1
        && block_picked == 0 && retreated_with_block == 0){
        task = 4;
        Serial.println("Picking up block.");
    }

    if (block_found == 1 && block_approached == 1
        && block_picked == 1 && retreated_with_block == 0 && journey == 0){
        task = 5;
        Serial.println("Retreating with block.");
    }

    if (block_found == 1 && block_approached == 1
        && block_picked == 1 && retreated_with_block == 1 && journey == 0){
        task = 6; // turn around
        Serial.println("Starting 180+.");
    }

    if (block_found == 1 && block_approached == 1
        && block_picked == 1 && retreated_with_block == 1 && journey == 1){
        task = 1; // Return line follow.
        if (main_loop_counter % print_freq == 0){
            Serial.println("Returning line follow.");
        }
    }

    if ((left_intxn_counter == 4 || right_intxn_counter == 4)){
        task = 7;
        Serial.println("Manuvering to target.");
    }

    if (block_placed == 1){
        task = 8;
        Serial.println("Return and reset.");
    }


}

void MovementControl::DummyMove(){
    LFDataRead();
    EdgeDetection();
    IntersectionDetection();
    LeftMotor->run(FORWARD);
    LeftMotor->setSpeed(ref_speed);
    RightMotor->run(FORWARD);
    RightMotor->setSpeed(ref_speed);
}

void MovementControl::PID(){

    P = -(L1LF_data - R1LF_data);
    I = pre_I + P * 0.001 * (current_time - PID_prev_time);
    PIDError = P * kp + I * ki;
    pre_I = I;
    PID_prev_time = current_time;

  speedL = ref_speed + PIDError;
  speedR = ref_speed - PIDError;

  if (speedL > max_speed) {
    speedL = max_speed;
  }
  if (speedR > max_speed) {
    speedR = max_speed;
  }
  if (speedL < 0) {
    speedL = 0;
  }
  if (speedR < 0) {
    speedR = 0;
  }
  LeftMotor->run(FORWARD);
  RightMotor->run(FORWARD);

  if (speedL != prev_speedL){
    LeftMotor->setSpeed(speedL);
    prev_speedL = speedL;
  }
  if (speedR != prev_speedR){
    RightMotor->setSpeed(speedR);
    prev_speedR = speedR;
  }
}
void MovementControl::LineFollow(){
    LFDataRead();
    EdgeDetection();
    IntersectionDetection();
    // Stray();
    PID();
    if (main_loop_counter % print_freq == 0){
        // Serial.println("Sensor 1 2: " + String(L1LF_data) + " " + String(R1LF_data));
        // Serial.println("PID value: "+ String(PIDError));
        Serial.println("Motor speed L R: " + String(speedL) + " "
         + String(speedR));
    }
}

void MovementControl::Search(){
    // First block straight approach line follow.

    int search_speed = 200;

    if ( block_number == 1 ){
        block_found = 1;
        return;
    }

    // Additional line follow after post-ramp intersection.
    long before_search_lf_time = millis();
    int before_search_lf_period = 3000;
    while (millis() - before_search_lf_time < before_search_lf_period){
        BlinkAmber();
        LineFollow();
    }

    // SWIVEL SEARCH.
    LeftMotor->setSpeed(0);
    RightMotor->setSpeed(0);
    delay(500);

    LeftMotor->run(RELEASE);
    RightMotor->run(RELEASE);

    int swivel_time = 1500;
    int sanity_delay = 300;

    // Swivel left.
    long search_start_time = millis();
    LeftMotor->run(BACKWARD);
    LeftMotor->setSpeed(search_speed);
    while(millis() - search_start_time < swivel_time && block_found != 1){
        delay(10);
    }
    LeftMotor->setSpeed(0);
    RightMotor->setSpeed(0);
    delay(sanity_delay);

    // Swivel back while searching.
    LeftMotor->run(FORWARD);
    LeftMotor->setSpeed(search_speed);
    search_start_time = millis();
    while (millis() - search_start_time < swivel_time && block_found != 1){
        BlockDetection();
        delay(10);
    }
    LeftMotor->setSpeed(0);
    RightMotor->setSpeed(0);

    if (block_found == 1){
        Serial.println("Found.");
        return;
    }
    delay(sanity_delay);

    // Swivel right.
    RightMotor->run(BACKWARD);
    search_start_time = millis();
    RightMotor->setSpeed(search_speed);
    while (millis() - search_start_time < swivel_time && block_found != 1){
        delay(10);
    }

    LeftMotor->setSpeed(0);
    RightMotor->setSpeed(0);
    delay(sanity_delay);

    // Swivel back while searching.
    RightMotor->run(FORWARD);
    search_start_time = millis();
    RightMotor->setSpeed(search_speed);
    while (millis() - search_start_time < swivel_time && block_found != 1){
        BlockDetection();
        delay(10);
    }

    LeftMotor->setSpeed(0);
    RightMotor->setSpeed(0);

    if (block_found == 1){
        Serial.println("Found.");
        return;
    }
    prev_speedL = 0;
    prev_speedR = 0;

    if (block_found == 0){
        digitalWrite(AmberLED, LOW);
        Serial.println("Didn't find.");
    }
    delay(2000);

    digitalWrite(AmberLED, HIGH);
    block_found = 1; // To be deleted.

}
void MovementControl::Approach(){

    if (block_number == 1){
        speedL = 0;
        speedR = 0;
        LeftMotor->setSpeed(speedL);
        RightMotor->setSpeed(speedR);
        prev_speedL = speedL;
        prev_speedR = speedR;
        ColorBaseline();
        long before_search_lf_time = millis();
        Serial.println("First block starting approach. Time:" + String(before_search_lf_time));
        while (millis() - before_search_lf_time < 4300){
            BlinkAmber();
            LineFollow();
        }
        Serial.println("First block approched. Time: " + String(millis()));
        block_approached = 1;

        approach_time = 1500;
        LeftMotor->setSpeed(0);
        RightMotor->setSpeed(0);
        prev_speedL = 0;
        prev_speedR = 0;
        delay(200);
        Color();
        return;
    }

    speedL = 0;
    speedR = 0;
    LeftMotor->setSpeed(speedL);
    RightMotor->setSpeed(speedR);
    prev_speedL = speedL;
    prev_speedR = speedR;
    ColorBaseline();

    // Converts distance sensor reading to time needed to approach the block.
    long k = 80000; 
    approach_time = k * (block_distance / ref_speed);
    Serial.println("Stopping to start approaching: " + String(approach_time));
    delay(2000);
    long approach_start_time = millis();
    LeftMotor->run(FORWARD);
    RightMotor->run(FORWARD);
    LeftMotor->setSpeed(ref_speed);
    RightMotor->setSpeed(ref_speed);
    while (millis() - approach_start_time <= approach_time){
        BlinkAmber();
    }
    LeftMotor->setSpeed(0);
    RightMotor->setSpeed(0);
    prev_speedL = 0;
    prev_speedR = 0;
    delay(200);
    Color();
    block_approached = 1;
    Serial.println("block_approached");
}
void MovementControl::Retreat(){
    long retreat_start_time = millis();

    LeftMotor->run(BACKWARD);
    RightMotor->run(BACKWARD);
    LeftMotor->setSpeed(ref_speed);
    RightMotor->setSpeed(ref_speed);

    while (millis() - retreat_start_time <= approach_time){
        BlinkAmber();
    }

    LeftMotor->setSpeed(0);
    RightMotor->setSpeed(0);
    prev_speedL = 0;
    prev_speedR = 0;
    retreated_with_block = 1;
}

void MovementControl::LineFindTurn(){
  if (Turn == 1)  //LEFT
  {
      digitalWrite(GreenLED, LOW);
      delay(10);
      digitalWrite(GreenLED, HIGH);
      delay(10);
      LeftMotor->run(BACKWARD);
      LeftMotor->setSpeed(turn_speed);
      RightMotor->run(FORWARD);
      RightMotor->setSpeed(turn_speed);
      delay(line_find_turn_delay); // turnaround guarantee.
      digitalWrite(RedLED, LOW);
      delay(10);
      digitalWrite(RedLED, HIGH);

      LFDataRead();
      while(L1LF_data != 1){
        BlinkAmber();
        LFDataRead();
      }
      delay(100);
      digitalWrite(GreenLED, LOW);
      delay(10);
      digitalWrite(GreenLED, HIGH);
      delay(10);
  }
  if (Turn == 2) // RIGHT
  {
      digitalWrite(RedLED, LOW);
      delay(10);
      digitalWrite(RedLED, HIGH);
      delay(10);
      LeftMotor->run(FORWARD);
      LeftMotor->setSpeed(turn_speed);
      RightMotor->run(BACKWARD);
      RightMotor->setSpeed(turn_speed);
      delay(line_find_turn_delay); // turnaround guarantee.
      digitalWrite(RedLED, LOW);
      delay(10);
      digitalWrite(RedLED, HIGH);

      LFDataRead();
      while(R1LF_data != 1){
        BlinkAmber();
        LFDataRead();
      }
      delay(100);
      digitalWrite(RedLED, LOW);
      delay(10);
      digitalWrite(RedLED, HIGH);
      delay(10);
  }
  LeftMotor->setSpeed(0);
  RightMotor->setSpeed(0);
  prev_speedL = 0;
  prev_speedR = 0;
  speedL = 0;
  speedR = 0;

  if (block_placed == 1 && journey == 1){
      journey = 0;
      Serial.println("Turn complete. Journey flipped 1 to 0.");
  }
  else if (journey == 0){
      journey = 1;
      Serial.println("Turn complete. Journey flipped 0 to 1.");
  }
  delay(500); // Sanity pause.
}

void MovementControl::LFDelivery(){
    Serial.println("Starting LF Delivery");
    long before_delivery_lf_time = millis();
    int before_delivery_forward_period = 3450; // This is pretty good.
    int before_delivery_retreat_period = 300;

    if (block_color == 0){
        if (far_blue_target == 0){
            Serial.println("Delivering FAR BLUE target.");
            far_blue_target = 1;
            while ( millis() - before_delivery_lf_time < before_delivery_forward_period){
                LineFollow();
            }
        }
        else if (far_blue_target == 1){
            Serial.println("Delivering CLOSE BLUE target.");
            close_blue_target = 1;
            LeftMotor->setSpeed(0);
            RightMotor->setSpeed(0);
            LeftMotor->run(BACKWARD);
            RightMotor->run(BACKWARD);
            LeftMotor->setSpeed(ref_speed);
            RightMotor->setSpeed(ref_speed);
            while ( millis() - before_delivery_lf_time < before_delivery_retreat_period ){
                delay(10);
            }
        }
    }
    if (block_color == 1){
        if (far_red_target == 0){
            Serial.println("Delivering FAR RED target.");
            far_red_target = 1;
            while ( millis() - before_delivery_lf_time < before_delivery_forward_period){
                LineFollow();
            }
        }
        else if (far_red_target == 1){
            Serial.println("Delivering CLOSE RED target.");
            far_red_target = 1;
            LeftMotor->setSpeed(0);
            RightMotor->setSpeed(0);
            LeftMotor->run(BACKWARD);
            RightMotor->run(BACKWARD);
            LeftMotor->setSpeed(ref_speed);
            RightMotor->setSpeed(ref_speed);
            while ( millis() - before_delivery_lf_time < before_delivery_retreat_period ){
                delay(10);
            }
        }
    }

    LeftMotor->setSpeed(0);
    RightMotor->setSpeed(0);
    delay(500);

    line_find_turn_delay = 1500;
    // HardTurn();
    LineFindTurn();

    long delivery_lf_start_time = millis();

    while(millis() - delivery_lf_start_time < delivery_lf_period){
        LineFollow();
    }

    LeftMotor->setSpeed(0);
    RightMotor->setSpeed(0);
    prev_speedL = 0;
    prev_speedR = 0;
    delay(1000);

    // minor adjustments
    Place();

    /*
    int current_left_intxn = left_intxn_counter;
    int current_right_intxn = right_intxn_counter;
    Serial.println("CURRENT counters L R: " + String(current_left_intxn) + " " + String(current_right_intxn));

    while((current_left_intxn == left_intxn_counter) && (current_right_intxn == right_intxn_counter)){
        Serial.println("Sensor Data: " + String(L2LF_data) + " " + String(L1LF_data)+" " + String(R1LF_data) + " " + String(R2LF_data));
        Serial.println("Intersection counters L R: " + String(left_intxn_counter) + " " + String(right_intxn_counter));
        EdgeDetection();
        IntersectionDetection();
        delay(100);
    }
    */
}
void MovementControl::Reset(){
    long reset_start_time = millis();

    LeftMotor->run(BACKWARD);
    RightMotor->run(BACKWARD);
    LeftMotor->setSpeed(ref_speed);
    RightMotor->setSpeed(ref_speed);

    while(millis() - reset_start_time < delivery_lf_period){
        delay(100);
    }

    LeftMotor->setSpeed(0);
    RightMotor->setSpeed(0);
    line_find_turn_delay = 1500;
    LineFindTurn();
    // return move.

    block_found = 0;
    block_approached = 0;
    block_picked = 0;
    retreated_with_block = 0;
    journey = 0;
    block_placed = 0;
    left_intxn_counter = 2;
    right_intxn_counter = 2;
    color_baseline = 0;
    if (block_number == 2){
        Serial.println("ending run");
        Finish();
    }
    Serial.println("Ready for new run.");
    delay(2000);
}

void MovementControl::Stray(){
    if ((stray_left <= stray_threshold) && (stray_left >= (-stray_threshold))){
        return;
    }
    LeftMotor -> setSpeed(0);
    RightMotor -> setSpeed(0);
    if (stray_left > stray_threshold){
        Serial.println("STRAYED LEFT!");
        RightMotor->run(BACKWARD);
        RightMotor->setSpeed(ref_speed);
        while(L1LF_data != 1){
            BlinkAmber();
            LFDataRead();
        }
        Serial.println("Stray left corrected.");
        stray_left = 0;
        LeftMotor -> setSpeed(0);
        RightMotor -> setSpeed(0);
        prev_speedL = 0;
        prev_speedR = 0;
        delay(500);

        return;
    }
    if (stray_left < (-stray_threshold)){
        Serial.println("STRAYED RIGHT!");
        LeftMotor->run(BACKWARD);
        LeftMotor->setSpeed(ref_speed);
        while(R1LF_data != 1){
            BlinkAmber();
            LFDataRead();
        }
        Serial.println("Stray right corrected.");
        stray_left = 0;
        LeftMotor -> setSpeed(0);
        RightMotor -> setSpeed(0);
        prev_speedL = 0;
        prev_speedR = 0;
        delay(500);
        return;
    }
    delay(500);
}

void MovementControl::Finish(){
    line_find_turn_delay = 3000; // 180 guarantee;
    LineFindTurn();
    int current_left_intxn = left_intxn_counter;
    int current_right_intxn = right_intxn_counter;
    while(current_left_intxn == left_intxn_counter && current_right_intxn == right_intxn_counter){
        LineFollow();
    }
    long before_finish_dummy_time = millis();
    int before_finish_dummy_period = 3000;
    while (millis() - before_finish_dummy_time < before_finish_dummy_period){
        DummyMove();
    }
    LeftMotor->setSpeed(0);
    RightMotor->setSpeed(0);
    while(true){
      delay(1000);
    }
}

void MovementControl::BlinkAmber(){
  if (millis() - prev_blink_time > 500){
    digitalWrite(AmberLED, LOW);
    delay(10);
    digitalWrite(AmberLED, HIGH);
    prev_blink_time = millis();
  }
}

void MovementControl::HardTurn(){
    if (Turn == 1)  //LEFT
    {
        LeftMotor->run(BACKWARD);
        LeftMotor->setSpeed(turn_speed);
        RightMotor->run(FORWARD);
        RightMotor->setSpeed(turn_speed);
        delay(2000); // turn time
    }
    if (Turn == 2) // RIGHT
    {
        LeftMotor->run(FORWARD);
        LeftMotor->setSpeed(turn_speed);
        RightMotor->run(BACKWARD);
        RightMotor->setSpeed(turn_speed);
        delay(2000);
    }
    LeftMotor->setSpeed(0);
    RightMotor->setSpeed(0);

}
void MovementControl::HardDelivery(){
    // This would be much better done with line follower.
    long before_delivery_lf_time = millis();
    int before_delivery_forward_period = 1000; // 1 more second before stop.
    int delivery_time = 3000;
    while (millis() - before_delivery_lf_time <= before_delivery_forward_period){
        LineFollow();
    }
    LeftMotor->setSpeed(0);
    RightMotor->setSpeed(0);
    prev_speedL = 0;
    prev_speedR = 0;
    HardTurn();
    // move and place.
    long second_stretch_start_time = millis();

    while (millis() - second_stretch_start_time < delivery_time){
        LeftMotor->setSpeed(ref_speed);
        RightMotor->setSpeed(ref_speed);
    }
    LeftMotor->setSpeed(0);
    RightMotor->setSpeed(0);
    prev_speedL = 0;
    prev_speedR = 0;
    Place();
}
void MovementControl::Stop(){
  LeftMotor->run(FORWARD);
  LeftMotor->setSpeed(0);
  RightMotor->run(FORWARD);
  RightMotor->setSpeed(0);
}

void setup()
{
  Serial.begin(9600);
  Serial.println("Program start.");
  AFMS.begin();

  for (int i=0;i<intxn_queue_length;i++)
  {
    L2Queue.enqueue(0);
    R2Queue.enqueue(0);
  }

  pinMode(GreenLED, OUTPUT);
  pinMode(RedLED, OUTPUT);
  pinMode(AmberLED, OUTPUT);

  pinMode(ButtonPin, INPUT);

  pinMode(L2LF_receive,INPUT);
  pinMode(L1LF_receive,INPUT);
  pinMode(R1LF_receive,INPUT);
  pinMode(R2LF_receive,INPUT);
  pinMode(A2, INPUT);

  clamp_servo.attach(clamp_servo_pin);
  chassis_servo.attach(chassis_servo_pin);
  clamp_servo.write(clamp_open);
  chassis_servo.write(chassis_high);

  digitalWrite(GreenLED, HIGH);
  digitalWrite(RedLED, HIGH);

}

void loop()
{

    if (main_loop_counter % print_freq == 0){
        Serial.println("Loop: " + String(main_loop_counter) + " ------------------------");
    }

    MovementControl MC;
    LFDetection LF;
    ServoMove SM;

    /* Record button presses. If pressed once, robot delivers 1 block only.
    If pressed twice, robot attempts all 4 blocks. */
    while (button_pressed == 0){
      LeftMotor->run(FORWARD);
      RightMotor->run(FORWARD);
      LeftMotor->setSpeed(0);
      RightMotor->setSpeed(0);
        buttonState = digitalRead(ButtonPin);
        Serial.println(buttonState);
        if (buttonState == 0) {
            button_pressed = 1;
            delay(2000);
        break;
        }
        delay(100);
    }

    current_time = millis();

    MC.FindTask();
    MC.BlinkAmber();

    switch ( task ){
        case 0:
            MC.DummyMove();
            break;
        case 1:
            MC.LineFollow();
            break;
        case 2:
            MC.Search();
            break;
        case 3:
            MC.Approach();
            break;
        case 4:
            SM.Pickup();
            break;
        case 5:
            MC.Retreat();
            break;
        case 6:
            line_find_turn_delay = 1000;
            MC.LineFindTurn();
            break;
        case 7:
            //MC.HardDelivery();
            MC.LFDelivery();
            break;
        case 8:
            MC.Reset();
            break;
    }

    delay(main_loop_delay_time);
    main_loop_counter++;
    if (main_loop_counter % print_freq == 0){
        Serial.println(" ");
    }
}
