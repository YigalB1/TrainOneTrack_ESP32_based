#include "WiFi.h"
#include <esp_now.h>

#define MY_NAME   "Train Motor controller"
//#define MY_ROLE         ESP_NOW_ROLE_CONTROLLER         // set the role of this device: CONTROLLER, SLAVE, COMBO
//#define RECEIVER_ROLE   ESP_NOW_ROLE_SLAVE              // set the role of the receiver
//#define WIFI_CHANNEL    1

// setting PWM properties
const int freq = 5000;
const int ledChannel = 0;
const int resolution = 10; //Resolution 8, 10, 12, 15


#define MOTOR_PWM 10
#define MOTOR_EN1 9
#define MOTOR_EN2 8
#define RED_LED   4
#define GREEN_LED 3
#define BLUE_LED  2
#define LED_CMD 2
#define LEDS_OFF_CMD 4
#define RED   1
#define GREEN 2
#define BLUE  3


const int ZERO = 0;
const int Num_Of_Slaves = 5;
const int Addr_Space = 127;
const int left_dev = 9;
const int right_dev = 10;
const int LEFT = 1;
const int RIGHT = 2;
const int UNKNOWN = 2;
const int STOP = 0;
const int ON_MOVE = 1;
const int VERY_CLOSE = 20;
const int CLOSE = 20;
const int IN_RANGE = 40;
const int MIN_SPEED = 120; // was 120
const int SLOW_SPEED = 140;
const int MAX_SPEED = 180; // was 180
const int SPEED_INC = 5;
const int TIME_IN_STATION = 5000;
const int SAMPLE_TIME = 200;
const int SPEEDING = 1;
const int SLOWING = 2;
const int STOPPING = 3;
const int JUNK_VAL = 7777;
const int DIST_BUFF_SIZE = 20;

int train_speed, L_distance, R_distance, train_direction, train_status;
int current_dev, current_dist;
unsigned long curr_time, time_prev;

const int DIST_ARRAY_SIZE = 100;
int dist_array[DIST_ARRAY_SIZE];
int time_array[DIST_ARRAY_SIZE];
int speed_array[DIST_ARRAY_SIZE];
int dist_cnt = 0;


// the packet stracture
struct __attribute__((packed)) dataPacket {
  int slave_num;
  int distance;
};

// called when a package is recieeved via ESPnow
// uint8_t
void dataReceived(const uint8_t *senderMac, const uint8_t *data, int dataLength) {
  char macStr[18];
  dataPacket packet;  

  snprintf(macStr, sizeof(macStr), "%02x:%02x:%02x:%02x:%02x:%02x", senderMac[0], senderMac[1], senderMac[2], senderMac[3], senderMac[4], senderMac[5]);

  Serial.println();
  Serial.print("Received data from: ");
  Serial.println(macStr);
  
  memcpy(&packet, data, sizeof(packet));
  
  Serial.print("slave_num: ");
  Serial.print(packet.slave_num);
  Serial.print("  / ");
  Serial.println(packet.distance);
}


class Motor {
  public:
    void Go ( int _direction, int _speed) {
      if (LEFT == _direction) {
        Go_left(_speed);
      }
      else {
        Go_right(_speed);
      }
    }

    void Go_left ( int _speed_) {
      digitalWrite(MOTOR_EN1, HIGH);
      digitalWrite(MOTOR_EN2, LOW);
      ledcWrite(ledChannel, _speed_);
    }

    void Go_right ( int _speed_) {
      digitalWrite(MOTOR_EN1, LOW);
      digitalWrite(MOTOR_EN2, HIGH);
      ledcWrite(ledChannel, _speed_);
    }
    void stop () {
      digitalWrite(MOTOR_EN1, LOW);
      digitalWrite(MOTOR_EN2, LOW);
      ledcWrite(ledChannel, ZERO);
    }

};


Motor train_motor;
int bad_reads_cnt = 0;
int good_reads_cnt = 0;
int dist_buffer[DIST_BUFF_SIZE];
int last_good_dist=IN_RANGE;

// functions headsup declare 
void decrease_speed() ;
void increase_speed() ;
void slow_down();
int read_distance(int dev);

// ******************* SETUP ***************/

void setup()
{
  int l0;
  
  pinMode(RED_LED, OUTPUT);
  pinMode(BLUE_LED, OUTPUT);
  pinMode(GREEN_LED, OUTPUT);

  ledcSetup(ledChannel, freq, resolution);
  ledcAttachPin(MOTOR_PWM, ledChannel);

  
  Serial.begin(9600);
  Serial.println();
  Serial.print("Initializing...");
  Serial.print(MY_NAME);
  Serial.print(".  My MAC address is: ");
  Serial.print(WiFi.macAddress());
  
  WiFi.mode(WIFI_STA);
  WiFi.disconnect();        // we do not want to connect to a WiFi network

  if(esp_now_init() != 0) {
    Serial.println("  ESP-NOW initialization failed");
    return;
    }

  //esp_now_set_self_role(ESP_NOW_ROLE_SLAVE);
  esp_now_register_recv_cb(dataReceived);   // this function will get called once all data is sent
  Serial.println("  ... ESPnow Initialized.");


  for (int i=0;i<DIST_BUFF_SIZE;i++) {
    dist_buffer[i]=  JUNK_VAL;
    }
  train_motor.stop(); // make sure it is stopped
  time_prev = millis();

 
  delay (1000);
  

 
  train_speed = 0;

  l0 = read_distance(left_dev);
  Serial.print("first read (left). l0: ");
  Serial.println(l0);

  if (l0 <= VERY_CLOSE) {
    train_direction = RIGHT;
    current_dev = right_dev;
    Serial.println("-> start with RIGHT");
  }
  else {
    train_direction = LEFT;
    current_dev = left_dev;
    Serial.println("-> start with LEFT");
  }

  Serial.println("** Still in SETUP");
  delay (1000);

  curr_time = millis();
  Serial.print(time_prev);
  Serial.print("//");
  Serial.print(curr_time);
  Serial.print("/delta/");
  Serial.print(curr_time - time_prev);
  Serial.println("//");

  for (int i = 0; i < DIST_ARRAY_SIZE; i++)
  {
    dist_array[i] = JUNK_VAL;
    time_array[i] = JUNK_VAL;
    speed_array[i] = JUNK_VAL;
  }
  train_speed = 0;
  train_motor.Go(train_direction, train_speed);

  Serial.println("******** End of SETUP ****");
}

// ******************* LOOP ***************/
// ****************************************/

void loop()
{

  //int distance;
  //boolean first;
//  unsigned long delta_time, t1, t2;
int dev; // other_dev was removed
  
// int first, last; // array pointers
//  delta_time = millis() - time_prev;

  if (RIGHT == train_direction)
    {
    dev = right_dev;
    Serial.print("R>> ");
    }
  else
    {
    dev = left_dev;
    Serial.print("L>> ");
    }


  if (current_dist < 2)
    {
    bad_reads_cnt += 1;
    return;
    }

  if (current_dist > 100)
    current_dist=101;
  
    good_reads_cnt += 1;

  for (int i=DIST_BUFF_SIZE-1;i>=1;i--)
    {
    dist_buffer[i] = dist_buffer[i-1];
    }
  dist_buffer[0] = current_dist;



  dist_array[dist_cnt] = current_dist;
//  time_array[dist_cnt] = t2 - t1;
//  speed_array[dist_cnt] = train_speed;
  dist_cnt++;
  if (DIST_ARRAY_SIZE == dist_cnt)
    dist_cnt = 0;

  // the train is assumed to be moving here
  // if it is in station, it will be in the fucntion, not here

  boolean stopping=false;
  if (current_dist < VERY_CLOSE )
    {
    train_motor.stop();
    train_speed=ZERO;
    stopping=true;
    }
  else if (current_dist < CLOSE )
    {
      slow_down();  
    }
    else if (current_dist < IN_RANGE )
      {
        decrease_speed();
      }
      else
        {
          // speed it up baby, out of range
          increase_speed();
        }
          
  
  if (!stopping)
    train_motor.Go(train_direction, train_speed);
  else {
      // the train is stopping
      // print the last dist readings
      for (int i=DIST_BUFF_SIZE-1;i>=0;i--)
        {
        Serial.print(dist_buffer[i]);
        Serial.print("..");
        dist_buffer[i]=JUNK_VAL;
        } // of for
    
   //  delay(5000); // wait 10 seconds 
      // change directions
      if (train_direction==LEFT)
        {
        train_direction=RIGHT;
//        slave.Turn_led_ON (right_dev, BLUE);
//        slave.Turn_leds_OFF(left_dev);
        } // of if
      else
        {
        train_direction=LEFT;
//        slave.Turn_led_ON (left_dev, BLUE);
//        slave.Turn_leds_OFF(right_dev);
        } // of else
    } // of else

  Serial.print("msg1: dist: ");
  Serial.print(current_dist);
  Serial.print(" , speed: ");
  Serial.print(train_speed);
  Serial.print(" good reads: ");
  Serial.print(good_reads_cnt);
  Serial.print(" bads: ");
  Serial.println(bad_reads_cnt);

  delay(SAMPLE_TIME);
  // delay(500); // for DEBUG


}

// *****************************************
// **************** END LOOP ***************
// *****************************************

// ******** FUNCTIONS ***************************************


// ****************** stop_the_train **********************


// ****************** SLOW_DOWN **********************
void slow_down() {
  train_speed = MIN_SPEED;
}
// ****************** increase_speed **********************
void increase_speed() {
  train_speed += SPEED_INC;
  if (train_speed > MAX_SPEED)
    train_speed = MAX_SPEED;
}
// ****************** decrease_speed **********************
void decrease_speed() {
  train_speed -= SPEED_INC;
  if (train_speed < MIN_SPEED)
    train_speed = MIN_SPEED;
}

// ****************** read_distance **********************
int read_distance(int dev)
{
  int dist;
  

  dist=77; // measure distance TBD

  
  return dist;
}
