// Includes
#include "mavlink/common/mavlink.h"        // Mavlink interface
#include "mavlink/common/mavlink_msg_obstacle_distance.h"
#include <TFMPlus.h>  // Include TFMini Plus Library v1.4.1
TFMPlus tfmP;         // Create a TFMini Plus object
// TF-Mini or TF-Luna
#define TFMINI_BAUDRATE 115200 // bauds
#define TFMINI_DATARATE 10.0f // ms

int potiValue = 0;
int FOV = 120; //multiple of res and even(res is 3 degree for the TF02-pro)
int lidarAngle = 0;
int messageAngle = 0;
int res = 3;
uint16_t distances[72];
  

// Stepper
#define POTI_PIN PA0
#define DIR_PIN PB15
#define STEP_PIN PA8
#define PULSE_PER_REV 1600 // PPR_motor * Gear_reduction * Microstepping = 200*3/2*16
#define TIME_PER_REV 600.0f // ms
const int TIME_PER_PULSE = (TIME_PER_REV * 1000.0f)/ PULSE_PER_REV; // us
const int PULSE_PER_DATAPOINT = (TFMINI_DATARATE * 1000.0f) / TIME_PER_PULSE;




// External communication
#define EXTERNAL_BAUDRATE 1500000 // 115200 is too slow for the obstacle_distance message, so I went for max speed :-)

char serial_buffer[15];



HardwareSerial Serial2(USART2);
// the setup function runs once when you press reset or power the board
void setup() {
  
  
  
  
  // Stepper
  pinMode(DIR_PIN, OUTPUT);
  pinMode(STEP_PIN, OUTPUT);
  
  digitalWrite(DIR_PIN,HIGH);
   

  // Serial ports
  Serial1.begin(EXTERNAL_BAUDRATE); // USB
  
  Serial2.begin(TFMINI_BAUDRATE); // TF mini
  tfmP.begin(&Serial2);
  
  flushSerial2();
  memset(distances, UINT16_MAX, sizeof(distances)); // Filling the distances array with UINT16_MAX
 
  
  
}

// Scanning fuction. Adapt to your needs!
int16_t tfDist = 0;    // Distance to object in centimeters
//int16_t tfFlux = 0;    // Strength or quality of return signal
//int16_t tfTemp = 0;    // Internal temperature of Lidar sensor chip
uint16_t pulses = 0;

void loop() {
  
  
  potiValue = analogRead(POTI_PIN);
  lidarAngle = map(potiValue, 185, 815, -90, 90);          // Adjust for the poti you use
  messageAngle = map(lidarAngle, -FOV/2, FOV/2, 0, FOV);
  
  if (lidarAngle <= -FOV/2) 
  {
    
    digitalWrite(DIR_PIN, LOW);
  }

  if (lidarAngle >= FOV/2)
  {
    pulses = 0;
    
    digitalWrite(DIR_PIN, HIGH);
  }
  
  digitalWrite(STEP_PIN, HIGH);
  delayMicroseconds(TIME_PER_PULSE);
  digitalWrite(STEP_PIN, LOW);
  delayMicroseconds(TIME_PER_PULSE);
  pulses++;

  
  if(lidarAngle%res == 0){ // get a distance reading for each res (3 degree) step
    tfmP.getData(tfDist);
     
    send_pos();
  }



}

void send_pos(){

//MAVLINK DISTANCE MESSAGE
  int sysid = 1;                   
  //< The component sending the message.
  int compid = 158;    

  uint64_t time_usec = 0; /*< Time since system boot*/
  uint8_t sensor_type = 0;
  distances[messageAngle/res] = tfDist-2.0f; //UINT16_MAX gets updated with actual distance values
  uint8_t increment = 3;
  uint16_t min_distance = 5; /*< Minimum distance the sensor can measure in centimeters*/
  uint16_t max_distance = 4000; /*< Maximum distance the sensor can measure in centimeters*/
  float increment_f = 0;
  float angle_offset = -FOV/2;
  uint8_t frame = 12;
  // Initialize the required buffers
  mavlink_message_t msg;
  uint8_t buf[MAVLINK_MAX_PACKET_LEN];

  // Pack the message
 mavlink_msg_obstacle_distance_pack(sysid,compid,&msg,time_usec,sensor_type,distances,increment,min_distance,max_distance,increment_f,angle_offset,frame);

  // Copy the message to the send buffer
  uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
  // Send the message (.write sends as bytes) 
  //delay(1);
  Serial1.write(buf, len);

}


// Flushes the INPUT serial buffer
void flushSerial2(){
  while(Serial2.available()){Serial2.read();}
}
