
#include "mavlink.h"        // Mavlink interface
#include "mavlink_msg_distance_sensor.h"
#include <Wire.h>
#include <NewPing.h>
const int MIN = 10;
const int MAX = 5000;
const int RMIN = 10;
const int RMAX = 350;

NewPing sonar0(3, 4, 300);
NewPing sonar1(5, 6, 300);
NewPing sonar2(7, 8, 300);
NewPing sonar3(9, 10, 300);
NewPing sonar4(11, 12, 100);

#define bRate 115200
#define NSensores 5
#define NDistancias     5

//Record to store the data of each sensor
struct Sensores {
  uint16_t Distancias[NDistancias]  = {0};
  uint16_t MediaDistancias          = 0;
  bool Cerca                        = false;
  bool Activo                       = false;
  unsigned long CompensarTime       = 0;
};

//The variables of each sensor are started
#define NSensores 4
Sensores Sensor[NSensores];



void setup()
{
  
  Serial.begin(bRate);
  Wire.begin();
  Serial.println ("setup");

}


void loop() {

  FSensores();
  command_heartbeat();
  command_distance_1();
  command_distance_2();
  command_distance_3();
  command_distance_4();

 
}

void FSensores() {
  ShiftArrays();
  MedirSensores();
  MediaDistancias();
 delay(10);

}

//Desplaza cada array de Distancias en una posición
void ShiftArrays() {
  for (uint8_t i = 0; i < NSensores; i++) {
    for (uint8_t j = NDistancias - 1; j > 0; j--) {
      Sensor[i].Distancias[j] = Sensor[i].Distancias[j - 1];
    }
  }
}

//==================================SENSORES=====================================//
//Se miden los sensores, y se colocan en la posición 0 de cada array
void MedirSensores() {
  Sensor[0].Distancias[0] = sonar0.ping_cm();
  Sensor[1].Distancias[0] = sonar1.ping_cm();
  Sensor[2].Distancias[0] = sonar2.ping_cm();
  Sensor[3].Distancias[0] = sonar3.ping_cm();
  Sensor[4].Distancias[0] = sonar4.ping_cm();
}

//The average of all the distances is performed. Los 0 are discarded
void MediaDistancias() {
  for (uint8_t i = 0; i < NSensores; i++) {
    int Total   = 0;
    uint8_t Num = 0;
    for (uint8_t j = 0; j < NDistancias; j++) {
      if (Sensor[i].Distancias[j] != 0  && Sensor[i].Distancias[j] < 300) {
        Total += Sensor[i].Distancias[j];
        Num += 1;
      }
    }
    if (Num > 3) {
      Sensor[i].MediaDistancias = Total / Num;
    } else {
      Sensor[i].MediaDistancias = 0;
    }
  }
  //Serial.print("\n\rDistancias: ");
 //Serial.print(Sensor[0].MediaDistancias);
 // Serial.print(",");
 // Serial.print(Sensor[1].MediaDistancias);
 // Serial.print(",");
 // Serial.print(Sensor[2].MediaDistancias);
 // Serial.print(",");
 // Serial.print(Sensor[3].MediaDistancias);
//  Serial.print(",");
 // Serial.print(Sensor[4].MediaDistancias);
 // Serial.print("cm\n\r");
}


void command_heartbeat() {

  //< ID 1 for this system
  int sysid = 100;                   
  //< The component sending the message.
  int compid = MAV_COMP_ID_PATHPLANNER;    
  
  // Define the system type, in this case ground control station
  uint8_t system_type =MAV_TYPE_GCS;
  uint8_t autopilot_type = MAV_AUTOPILOT_INVALID;
  
  uint8_t system_mode = 0; 
  uint32_t custom_mode = 0;                
  uint8_t system_state = 0;
  
  // Initialize the required buffers
  mavlink_message_t msg;
  uint8_t buf[MAVLINK_MAX_PACKET_LEN];
  
  // Pack the message
  mavlink_msg_heartbeat_pack(sysid,compid, &msg, system_type, autopilot_type, system_mode, custom_mode, system_state);
  
  // Copy the message to the send buffer
  uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
  
  // Send the message 
  //delay(1);
  Serial.write(buf, len);
}


void command_distance_1() {

// READ THE DISTANCE SENSOR
  float Sensor1Smooth  = Sensor[0].MediaDistancias;
  Sensor1Smooth = constrain(Sensor1Smooth, MIN , MAX);
  float dist1 = Sensor1Smooth / Scale;
  
  //MAVLINK DISTANCE MESSAGE
  int sysid = 1;                   
  //< The component sending the message.
  int compid = 158;    

  uint32_t time_boot_ms = 0; /*< Time since system boot*/
  uint16_t min_distance = RMIN; /*< Minimum distance the sensor can measure in centimeters*/
  uint16_t max_distance = RMAX; /*< Maximum distance the sensor can measure in centimeters*/
  uint16_t current_distance = dist1; /*< Current distance reading*/
  uint8_t type = 0; /*< Type from MAV_DISTANCE_SENSOR enum.*/
  uint8_t id = 1; /*< Onboard ID of the sensor*/
  uint8_t orientation = 0; /*(0=forward, each increment is 45degrees more in clockwise direction), 24 (upwards) or 25 (downwards)*/
// Consumed within ArduPilot by the proximity class

  uint8_t covariance = 0; /*< Measurement covariance in centimeters, 0 for unknown / invalid readings*/


  // Initialize the required buffers
  mavlink_message_t msg;
  uint8_t buf[MAVLINK_MAX_PACKET_LEN];

  // Pack the message
 mavlink_msg_distance_sensor_pack(sysid,compid,&msg,time_boot_ms,min_distance,max_distance,current_distance,type,id,orientation,covariance);

  // Copy the message to the send buffer
  uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
  // Send the message (.write sends as bytes) 
  //delay(1);
  Serial.write(buf, len);
}


void command_distance_2() {

// READ THE DISTANCE SENSOR
  float Sensor2Smooth  = Sensor[1].MediaDistancias;
  Sensor2Smooth = constrain(Sensor2Smooth, MIN , MAX);
  float dist2 = Sensor2Smooth / Scale;

  //MAVLINK DISTANCE MESSAGE
  int sysid = 1;                   
  //< The component sending the message.
  int compid = 158;    

  uint32_t time_boot_ms = 0; /*< Time since system boot*/
  uint16_t min_distance = RMIN; /*< Minimum distance the sensor can measure in centimeters*/
  uint16_t max_distance = RMAX; /*< Maximum distance the sensor can measure in centimeters*/
  uint16_t current_distance = dist2; /*< Current distance reading*/
  uint8_t type = 0; /*< Type from MAV_DISTANCE_SENSOR enum.*/
  uint8_t id = 2; /*< Onboard ID of the sensor*/
  uint8_t orientation = 1; /*(0=forward, each increment is 45degrees more in clockwise direction), 24 (upwards) or 25 (downwards)*/
// Consumed within ArduPilot by the proximity class

  uint8_t covariance = 0; /*< Measurement covariance in centimeters, 0 for unknown / invalid readings*/


  // Initialize the required buffers
  mavlink_message_t msg;
  uint8_t buf[MAVLINK_MAX_PACKET_LEN];

  // Pack the message
 mavlink_msg_distance_sensor_pack(sysid,compid,&msg,time_boot_ms,min_distance,max_distance,current_distance,type,id,orientation,covariance);

  // Copy the message to the send buffer
  uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
  // Send the message (.write sends as bytes) 
  //delay(1);
  Serial.write(buf, len);
}


void command_distance_3() {

// READ THE DISTANCE SENSOR
  float Sensor3Smooth  = Sensor[2].MediaDistancias;
  Sensor3Smooth = constrain(Sensor3Smooth, MIN , MAX);
  float dist3 = Sensor3Smooth / Scale;

  //MAVLINK DISTANCE MESSAGE
  int sysid = 1;                   
  //< The component sending the message.
  int compid = 158;    

  uint32_t time_boot_ms = 0; /*< Time since system boot*/
  uint16_t min_distance = RMIN; /*< Minimum distance the sensor can measure in centimeters*/
  uint16_t max_distance = RMAX; /*< Maximum distance the sensor can measure in centimeters*/
  uint16_t current_distance = dist3; /*< Current distance reading*/
  uint8_t type = 0; /*< Type from MAV_DISTANCE_SENSOR enum.*/
  uint8_t id = 3; /*< Onboard ID of the sensor*/
  uint8_t orientation = 2; /*(0=forward, each increment is 45degrees more in clockwise direction), 24 (upwards) or 25 (downwards)*/
// Consumed within ArduPilot by the proximity class

  uint8_t covariance = 0; /*< Measurement covariance in centimeters, 0 for unknown / invalid readings*/


  // Initialize the required buffers
  mavlink_message_t msg;
  uint8_t buf[MAVLINK_MAX_PACKET_LEN];

  // Pack the message
 mavlink_msg_distance_sensor_pack(sysid,compid,&msg,time_boot_ms,min_distance,max_distance,current_distance,type,id,orientation,covariance);

  // Copy the message to the send buffer
  uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
  // Send the message (.write sends as bytes) 
  //delay(1);
  Serial.write(buf, len);
}



void command_distance_4() {

// READ THE DISTANCE SENSOR
  float Sensor4Smooth  = Sensor[3].MediaDistancias;
  Sensor4Smooth = constrain(Sensor4Smooth, MIN , MAX);
  float dist4 = Sensor4Smooth / Scale;

  //MAVLINK DISTANCE MESSAGE
  int sysid = 1;                   
  //< The component sending the message.
  int compid = 158;    

  uint32_t time_boot_ms = 0; /*< Time since system boot*/
  uint16_t min_distance = RMIN; /*< Minimum distance the sensor can measure in centimeters*/
  uint16_t max_distance = RMAX; /*< Maximum distance the sensor can measure in centimeters*/
  uint16_t current_distance = dist4; /*< Current distance reading*/
  uint8_t type = 0; /*< Type from MAV_DISTANCE_SENSOR enum.*/
  uint8_t id = 4; /*< Onboard ID of the sensor*/
  uint8_t orientation = 3; /*(0=forward, each increment is 45degrees more in clockwise direction), 24 (upwards) or 25 (downwards)*/
// Consumed within ArduPilot by the proximity class

  uint8_t covariance = 0; /*< Measurement covariance in centimeters, 0 for unknown / invalid readings*/


  // Initialize the required buffers
  mavlink_message_t msg;
  uint8_t buf[MAVLINK_MAX_PACKET_LEN];

  // Pack the message
 mavlink_msg_distance_sensor_pack(sysid,compid,&msg,time_boot_ms,min_distance,max_distance,current_distance,type,id,orientation,covariance);

  // Copy the message to the send buffer
  uint16_t len = mavlink_msg_to_send_buffer(buf, &msg);
  // Send the message (.write sends as bytes) 
  //delay(1);
  Serial.write(buf, len);
}
