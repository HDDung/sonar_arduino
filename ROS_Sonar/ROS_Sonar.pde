/*
 * rosserial SRF08 Ultrasonic Ranger Example
 *
 * This example is calibrated for the SRF08 Ultrasonic Ranger.
 */
#include <Sonar_srf08.h> //SRF08 specific library
#include <WProgram.h>
#include <Wire.h>
#include <ros.h>
#include <std_msgs/Float32MultiArray.h>
#include <std_msgs/String.h>

#define TIME_OUT 1000


/* Sonar_sfr08 Functions---
 	void connect();
 	void sendCommand(int commandRegister, int address, int command);
 	void setUnit(int commandRegister, int address, char units);
 	void setRegister(int address, int thisRegister);
 	int readData(int address, int numBytes);
 	void changeAddress(int commandRegister,int rear_left_addr);
 */

//Set up the ros node and publisher

std_msgs::Float32MultiArray sonar_msg;
//std_msgs::String sonarData;
ros::Publisher pub_sonar("sonar", &sonar_msg);
ros::NodeHandle nh;


char temp_string[50];


#define CommandRegister 0x00
#define ResultRegister  0x02


float sensorReading = 0;
char unit = 'c'; // 'i' for inches , 'c' for centimeters

Sonar_srf08 Sensor_1;
int Sensor_1_addr = 0xE4;

Sonar_srf08 Sensor_2;
int Sensor_2_addr = 0xE6;

Sonar_srf08 Sensor_3;
int Sensor_3_addr = 0xE8;

Sonar_srf08 Sensor_4;
int Sensor_4_addr = 0xEA;


void setup()
{
  Sensor_1.connect();  
  Sensor_2.connect();  
  Sensor_3.connect();  
  Sensor_4.connect();  

  //rear_centre.changeAddress(CommandRegister, rear_left_addr);

  Sensor_1_addr = Sensor_1_addr >> 1;
  Sensor_2_addr = Sensor_2_addr >> 1;
  Sensor_3_addr = Sensor_3_addr >> 1;
  Sensor_4_addr = Sensor_4_addr >> 1;

  nh.initNode();

  sonar_msg.layout.dim = (std_msgs::MultiArrayDimension *)malloc(sizeof(std_msgs::MultiArrayDimension));
  sonar_msg.layout.dim[0].label = "sonar";
  sonar_msg.layout.dim[0].size = 4;
  sonar_msg.layout.dim[0].stride = 1*4;
  //sonar_msg.layout.dim_length = 1;
  sonar_msg.layout.data_offset = 0;

  sonar_msg.data = (float *)malloc(sizeof(float)*4);
  sonar_msg.data_length = 4;

  nh.advertise(pub_sonar);
  
  //request reading from sensor
  Sensor_1.setUnit(CommandRegister, Sensor_1_addr, unit);
  Sensor_2.setUnit(CommandRegister, Sensor_2_addr, unit);
  Sensor_3.setUnit(CommandRegister, Sensor_3_addr, unit);
  Sensor_4.setUnit(CommandRegister, Sensor_4_addr, unit);

  //pause
  delay(70);

}


void loop()
{

  //set register for reading
  Sensor_1.setRegister(Sensor_1_addr, ResultRegister);
  Sensor_2.setRegister(Sensor_2_addr, ResultRegister);
  Sensor_3.setRegister(Sensor_3_addr, ResultRegister);
  Sensor_4.setRegister(Sensor_4_addr, ResultRegister);

  //read data from result register
  sonar_msg.data[0] = Sensor_1.readData(Sensor_1_addr, 2, TIME_OUT);
  sonar_msg.data[1] = Sensor_2.readData(Sensor_2_addr, 2, TIME_OUT);
  sonar_msg.data[2] = Sensor_3.readData(Sensor_3_addr, 2, TIME_OUT);
  sonar_msg.data[3] = Sensor_4.readData(Sensor_4_addr, 2, TIME_OUT);

  //Publish the data from sonars
  pub_sonar.publish(&sonar_msg);

  nh.spinOnce();
}
