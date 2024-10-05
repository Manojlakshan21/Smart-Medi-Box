#include <WiFi.h>
#include <PubSubClient.h>
#include "DHTesp.h"
#include <NTPClient.h>
#include <WiFiUdp.h>
#include <ESP32Servo.h>

#define DHT_PIN 15
#define BUZZER 12
#define LDR1 34
#define LDR2 35
#define MOTOR 18


Servo motor;

WiFiClient espClient; 

PubSubClient mqttClient(espClient); // for interacting with the MQTT broker.

WiFiUDP ntpUDP; // for NTP communication.

NTPClient timeClient(ntpUDP); // for synchronizing time.

DHTesp dhtSensor; //for temperature and humidity readings.

float minAngle=30.0;// Minimum angle of the shaded sliding window
float controlFac=0.75;// Controlling factor used to calculate motor angle
float customMinAngle;
float customControlFac;

// pre define values for commonly used medicines
float minAngle_A = 20;
float controlFac_A = 0.3;
float minAngle_B = 30;
float controlFac_B = 0.65;
float minAngle_C = 25;
float controlFac_C = 0.5;
bool custom = false;

bool isScheduledON = false;// Flag to indicate if the schedule is enabled
unsigned long scheduledOnTime;

char tempAr[6];// Array to store temperature as a string
char humidAr[6];// Array to store humidity as a string
char lightAr[6];// Array to store light intensity as a string

void setup() {
  Serial.begin(115200);  
  setupWifi();// Connect to WiFi network

  setupMqtt();// Setup MQTT communication

  dhtSensor.setup(DHT_PIN, DHTesp::DHT22);// Setup DHT sensor

  timeClient.begin();
  timeClient.setTimeOffset(5.5*3600);// Set time offset (5.5 hours for Sri Lanka)

  pinMode(BUZZER, OUTPUT);
  digitalWrite(BUZZER, LOW);
  
  pinMode(LDR1, INPUT);
  pinMode(LDR2, INPUT);

  motor.attach(MOTOR, 500, 2400);// Attach servo motor to pin
}


void loop() {
  if (!mqttClient.connected()){
    connectToBroker();// Connect to MQTT broker if not already connected
  }

  mqttClient.loop();

  updateTemperatureAndHumidity();// Read temperature from DHT sensor
  Serial.println(tempAr);
  Serial.println(humidAr);
  mqttClient.publish("MED-TEMP",tempAr);// Publish temperature to MQTT topic
  mqttClient.publish("MED-HUMD",humidAr);

  checkSchedule();// Check if the scheduled time has arrived

  updateLightIntensity();// Read light intensity from LDR  
  mqttClient.publish("LIGHT-INT",lightAr);// Publish light intensity to MQTT topic

  delay(1000);
}

void buzzerOn(bool on){
  if (on){
    tone(BUZZER, 256);
  }
  else{
    noTone(BUZZER);
  }
}

void setupMqtt(){
  mqttClient.setServer("test.mosquitto.org",1883);
  mqttClient.setCallback(receiveCallback);
}

void setupWifi(){
  Serial.println();
  Serial.print("Connecting to ");
  Serial.println("Wokwi-GUEST");
  WiFi.begin("Wokwi-GUEST","");

  while (WiFi.status()!=WL_CONNECTED){
    delay(500);
    Serial.print(".");
  }

  Serial.println("Wifi connected");
  Serial.print("IP Address: ");
  Serial.println(WiFi.localIP());
}

void receiveCallback(char* topic, byte* payload, unsigned int length){
  Serial.print("Message arrived [");
  Serial.print(topic);
  Serial.print("] ");

  char payloadCharAr[length];

  for (int i=0;i<length;i++){
    Serial.print((char)payload[i]);
    payloadCharAr[i]=(char)payload[i];
  }

  Serial.println(":");

  if (strcmp(topic,"MAIN-ON-OFF-MNL")==0){  
      buzzerOn(payloadCharAr[0]=='1');
  }
  else if (strcmp(topic,"ADMIN-SCH-ON")==0){
      if (payloadCharAr[0]=='N'){
        isScheduledON = false;
      }else{
        isScheduledON = true;
        scheduledOnTime = atol(payloadCharAr);
      }
  }
  if (strcmp(topic,"MINIMUM-ANG-MNL")==0){
    customMinAngle = atof(payloadCharAr);
    Serial.println(customMinAngle);
          
  }
  if (strcmp(topic,"CONTROL-FAC-MNL")==0){
    customControlFac = atof(payloadCharAr);
    Serial.println(customControlFac);
              
  }

  if (strcmp(topic,"DROP-DOWN-MNL")==0){
      if (payloadCharAr[0]=='A'){
        minAngle = minAngle_A;
        controlFac = controlFac_A;
        Serial.println(minAngle);
        Serial.println(controlFac);
      }else if(payloadCharAr[0]=='B'){
        minAngle = minAngle_B;
        controlFac = controlFac_B;
        Serial.println(minAngle);
        Serial.println(controlFac);
      }else if(payloadCharAr[0]=='C'){
        minAngle = minAngle_C;
        controlFac = controlFac_C;
        Serial.println(minAngle);
        Serial.println(controlFac);
      }
      else if(payloadCharAr[0]=='M'){
        custom = true;
      }

      
  }


}

void connectToBroker(){
  while (!mqttClient.connected()){
    Serial.println("Attempting MQTT connetion...");
    if (mqttClient.connect("ESP-9813247900")){
      Serial.println("Connected");
      mqttClient.subscribe("MAIN-ON-OFF-MNL");
      mqttClient.subscribe("ADMIN-SCH-ON");
      mqttClient.subscribe("DROP-DOWN-MNL");
      mqttClient.subscribe("MINIMUM-ANG-MNL");
      mqttClient.subscribe("CONTROL-FAC-MNL");
      
    }
    else{
      Serial.println("failed");
      Serial.println(mqttClient.state());
      delay(5000);
    }
  }
}

void updateTemperatureAndHumidity(){
  TempAndHumidity data=dhtSensor.getTempAndHumidity(); // get the sensor data
  String(data.temperature,2).toCharArray(tempAr, 6); // convert temperature value into char array
  String(data.humidity,2).toCharArray(humidAr, 6); // convert humidity value into char array
}

void updateLightIntensity() {
  const float analogMinValue = 0.0;   // Minimum analogRead value
  const float analogMaxValue = 1023.0; // Maximum analogRead value
  const float intensityMin = 0.0;  // Minimum intensity value
  const float intensityMax = 1.0;  // Maximum intensity value

  float rightLDR = analogRead(LDR1); // get analog out put of the LDR1
  float leftLDR = analogRead(LDR2); // get analog out put of the LDR2
  float intensity = 0 ;

  if(rightLDR > leftLDR){ // check whether right LDR give maximum value
    if(rightLDR <= 1023){ // check whether the output value exceed the maximum value
      intensity = (rightLDR-analogMinValue)/(analogMaxValue-analogMinValue);
      Serial.println("rightLDR "+String(rightLDR)+"  "+String(intensity));
      mqttClient.publish("MAX-LIGHT-INTENSITY","RIGHT LDR");
      String(intensity, 2).toCharArray(lightAr, 6);
      AdjustServoMotor(intensity,0.5);// Adjust the position of the shaded sliding window based on light intensity

    }
    else{
      intensity = 1;
      String(intensity, 2).toCharArray(lightAr, 6);
      Serial.println("rightLDR "+String(rightLDR)+"  "+String(intensity));
      mqttClient.publish("MAX-LIGHT-INTENSITY","RIGHT LDR");
      AdjustServoMotor(intensity,0.5);// Adjust the position of the shaded sliding window based on light intensity

    
    }
} 
  else{
    if(leftLDR <= 1023){ 
      intensity = (leftLDR-analogMinValue)/(analogMaxValue-analogMinValue);
      Serial.println("leftLDR "+String(leftLDR)+"  "+String(intensity));
      mqttClient.publish("MAX-LIGHT-INTENSITY","LEFT LDR");
      String(intensity, 2).toCharArray(lightAr, 6);
      AdjustServoMotor(intensity,1.5);
    }
    else{
      intensity = 1;
      String(intensity, 2).toCharArray(lightAr, 6);
      Serial.println("leftLDR "+String(leftLDR)+"  "+String(intensity));
      mqttClient.publish("MAX-LIGHT-INTENSITY","LEFT LDR");
      AdjustServoMotor(intensity,1.5);// Adjust the position of the shaded sliding window based on light intensity


    }
    // Adjust the position of the shaded sliding window based on light intensity
  }
    
}

unsigned long getTime(){
  timeClient.update();
  return timeClient.getEpochTime();// Get current time from NTP server
}

void checkSchedule(){
  if (isScheduledON){
    unsigned long currentTime = getTime();
    Serial.println("currentTime "+String(currentTime));
    Serial.println("scheduledOnTime "+String(scheduledOnTime));
    if (currentTime > scheduledOnTime){
      buzzerOn(true);
      isScheduledON = false;
      mqttClient.publish("ADMIN-MAIN-ON-OFF-ESP","1");
      mqttClient.publish("SCH-ESP-ON-MNL","0");
      Serial.println("Scheduled ON");
    }
  }  
}



void AdjustServoMotor(double lightintensity,double D){
  double angle;
  if(custom){
    angle = customMinAngle*D +(180.0-customMinAngle)*lightintensity*customControlFac;// Calculate the angle based on light intensity
  }
  else{
    angle = minAngle*D +(180.0-minAngle)*lightintensity*controlFac;// Calculate the angle based on light intensity
  }
  
  Serial.println(angle);
  motor.write(angle);// Set the angle of the servo motor to adjust the shaded sliding window  
}