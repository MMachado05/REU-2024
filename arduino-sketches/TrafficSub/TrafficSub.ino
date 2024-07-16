#include <arduino.h>
#include <Wire.h>
#include <ESP8266WiFi.h>
#include <ros.h>
#include <std_msgs/Bool.h>

// Allows connection to relay
#define i2c_address 0x27

// Light timing durations in milliseconds
int greenDuration = 5000;
int yellowDuration = 2000;
int redDuration = 5000;
int overlapRedDuration = 2000;

// Relay assignments
#define NORTH_GREEN 5
#define NORTH_YELLOW 4
#define NORTH_RED 3
#define SOUTH_GREEN 15
#define SOUTH_YELLOW 16
#define SOUTH_RED 2
#define EAST_GREEN 12
#define EAST_YELLOW 13
#define EAST_RED 14
#define WEST_GREEN 11
#define WEST_YELLOW 10
#define WEST_RED 9

// Direction phases
#define NS_GREEN 1
#define NS_YELLOW 2
#define EW_GREEN 3
#define EW_YELLOW 4
#define ALL_RED 5
#define N_GREEN 6
#define N_YELLOW 7
#define S_GREEN 8
#define S_YELLOW 9

// RSU Connection variables
IPAddress ip(10, 42, 0, 3);
IPAddress server(10, 42, 0, 3);
uint16_t serverPort = 11411;
const char*  ssid = "Roadside Unit";
const char*  password = "teach4990";

ros::NodeHandle nh;

bool northState = false;
bool southState = false;
bool notConnected = false;

void setupWiFi();
void initializeRelays();
void northCallback(const std_msgs::Bool& msg);
void southCallback(const std_msgs::Bool& msg);

ros::Subscriber<std_msgs::Bool> north("/light/northbound/state", &northCallback);
ros::Subscriber<std_msgs::Bool> south("/light/eastbound/state", &southCallback);


void northCallback(const std_msgs::Bool& msg) {
  // Print the received message
  Serial.print("Received message: ");
  Serial.println(msg.data);
  // activateNorth(msg.data);
  if (msg.data) {
    northState = true;
  }
  else {
    northState = false;
  }
}

void southCallback(const std_msgs::Bool& msg) {
  // Print the received message
  Serial.print("Received message: ");
  Serial.println(msg.data);
  // activateNorth(msg.data);
  if (msg.data) {
    southState = true;
  }
  else {
    southState = false;
  }
}


void setup() {
  // put your setup code here, to run once:
  Wire.begin();
  Serial.begin(115200);
  setupWiFi();

  nh.getHardware()->setConnection(server, serverPort);
  nh.initNode();

  nh.subscribe(north);
  nh.subscribe(south);

  Serial.print("ROS IP = ");
  Serial.println(nh.getHardware()->getLocalIP());

  initializeRelays();

}

void loop() {
  
  notConnected = false;
  if (!nh.connected()){
    Serial.println("Not Connected");
    notConnected = true;
  }
  // put your main code here, to run repeatedly:
  nh.spinOnce();

  activateLight();

  delay(500);

}

void activateLight() {
  // Define activation patterns for each direction
  uint16_t relayState = 0xFFFF;

  if (notConnected) {
    relayState &= ~((1 << (NORTH_YELLOW - 1)) | (1 << (SOUTH_YELLOW - 1)));
    relayState &= ~((1 << (WEST_YELLOW - 1)) | (1 << (EAST_YELLOW - 1)));
  }
  else {
    if (northState && southState) {
      relayState &= ~((1 << (NORTH_GREEN - 1)) | (1 << (SOUTH_GREEN - 1)));
    }
    else if (northState && !southState){
      relayState &= ~((1 << (NORTH_GREEN - 1)) | (1 << (SOUTH_RED - 1)));
    }
    else if (!northState && southState){
      relayState &= ~((1 << (NORTH_RED - 1)) | (1 << (SOUTH_GREEN - 1)));
    }
    else {
      relayState &= ~((1 << (NORTH_RED - 1)) | (1 << (SOUTH_RED - 1)));
    }
  }

  Wire.beginTransmission(i2c_address);
  Wire.write(relayState & 0xFF); // Lower byte
  Wire.write((relayState >> 8) & 0xFF); // Upper byte
  Wire.endTransmission();

}

void initializeRelays() {
  // Ensure all relays are off to start
  Wire.beginTransmission(i2c_address);
  Wire.write(0xFF);
  Wire.write(0xFF);
  Wire.endTransmission();
}

void setupWiFi()
{  
   WiFi.begin(ssid, password);
   while (WiFi.status() != WL_CONNECTED) { delay(500);Serial.print("."); }
   Serial.print("SSID: ");
   Serial.println(WiFi.SSID());
   Serial.print("IP:   ");
   Serial.println(WiFi.localIP());
}
