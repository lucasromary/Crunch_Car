#include <Arduino.h>
#include <QTRSensors.h>
#include <Ultrasonic.h>
#include <SPI.h>
#include <Adafruit_MotorShield.h>

#include <WiFi.h>
#include <ESPmDNS.h>
#include <WiFiUdp.h>
#include <ArduinoOTA.h>

const char *ssid = "crunchlab";
const char *password = "crunchlab";
unsigned long previousMillis;

// Entre 0 et 1
#define SPEED_FACTOR 0.5
#define Kp 0.05
#define Ki 1
#define Kd 2.5

/*
  MOTOR
*/
Adafruit_MotorShield AFMS = Adafruit_MotorShield();

Adafruit_DCMotor *myMotor1 = AFMS.getMotor(1);
Adafruit_DCMotor *myMotor2 = AFMS.getMotor(2);
Adafruit_DCMotor *myMotor3 = AFMS.getMotor(3);
Adafruit_DCMotor *myMotor4 = AFMS.getMotor(4);

Adafruit_DCMotor tab[] = {*myMotor1, *myMotor2, *myMotor3, *myMotor4};
int speed_gauche = 0;
int speed_droite = 0;
/*
  LINE FOLLOWER
*/
QTRSensors qtr;

const uint8_t SensorCount = 8;
uint16_t sensorValues[SensorCount];
int last_error = 0;
int error = 0;

/*
  ULTRASON
*/
Ultrasonic ultrasonic(17, 16);
int distance;

void initOTA()
{
  // Port defaults to 3232
  // ArduinoOTA.setPort(3232);

  // Hostname defaults to esp3232-[MAC]
  ArduinoOTA.setHostname("ESP32");

  // No authentication by default
  // ArduinoOTA.setPassword("admin");

  // Password can be set with it's md5 value as well
  // MD5(admin) = 21232f297a57a5a743894a0e4a801fc3
  // ArduinoOTA.setPasswordHash("21232f297a57a5a743894a0e4a801fc3");

  ArduinoOTA
      .onStart([]()
               {
    String type;
    if (ArduinoOTA.getCommand() == U_FLASH)
      type = "sketch";
    else // U_SPIFFS
      type = "filesystem";

    // NOTE: if updating SPIFFS this would be the place to unmount SPIFFS using SPIFFS.end()
    Serial.println("Start updating " + type); })
      .onEnd([]()
             { Serial.println("\nEnd"); })
      .onProgress([](unsigned int progress, unsigned int total)
                  { Serial.printf("Progress: %u%%\r", (progress / (total / 100))); })
      .onError([](ota_error_t error)
               {
    Serial.printf("Error[%u]: ", error);
    if (error == OTA_AUTH_ERROR) Serial.println("Auth Failed");
    else if (error == OTA_BEGIN_ERROR) Serial.println("Begin Failed");
    else if (error == OTA_CONNECT_ERROR) Serial.println("Connect Failed");
    else if (error == OTA_RECEIVE_ERROR) Serial.println("Receive Failed");
    else if (error == OTA_END_ERROR) Serial.println("End Failed"); });

  ArduinoOTA.begin();
}

void calibrationLine()
{
  pinMode(LED_BUILTIN, OUTPUT);
  digitalWrite(LED_BUILTIN, HIGH); // turn on Arduino's LED to indicate we are in calibration mode

  // 2.5 ms RC read timeout (default) * 10 reads per calibrate() call
  // = ~25 ms per calibrate() call.
  // Call calibrate() 400 times to make calibration take about 10 seconds.
  for (uint16_t i = 0; i < 400; i++)
  {
    qtr.calibrate();
  }
  digitalWrite(LED_BUILTIN, LOW); // turn off Arduino's LED to indicate we are through with calibration

  // print the calibration minimum values measured when emitters were on

  for (uint8_t i = 0; i < SensorCount; i++)
  {
    Serial.print(qtr.calibrationOn.minimum[i]);
    Serial.print(' ');
  }
  Serial.println();

  // print the calibration maximum values measured when emitters were on
  for (uint8_t i = 0; i < SensorCount; i++)
  {
    Serial.print(qtr.calibrationOn.maximum[i]);
    Serial.print(' ');
  }
  Serial.println();
  Serial.println();
}

int getPositionLine()
{
  uint16_t position = qtr.readLineBlack(sensorValues);
/*
  for (uint8_t i = 0; i < SensorCount; i++)
  {
    Serial.print(sensorValues[i]);
    Serial.print('\t');
  }
  */
  Serial.print(position);
  return position;
}

int getDistanceUS()
{
  distance = ultrasonic.read();
  Serial.print(" Distance in CM US: ");
  Serial.println(distance);
  return distance;
}

void turn(int speed_gauche, int speed_droite, int sens)
{
  uint8_t i = 0;
  for (int j = 0; j < 4; j++)
  {
    tab[j].run(sens);
  }
  tab[0].setSpeed(speed_gauche * SPEED_FACTOR);
  tab[1].setSpeed(speed_gauche * SPEED_FACTOR);
  tab[2].setSpeed(speed_droite * SPEED_FACTOR);
  tab[3].setSpeed(speed_droite * SPEED_FACTOR);
}

void LineFollower(int position)
{
  if (position > 3500)
  {
    speed_droite = 255 - (position - 3500) / 13.73;
    speed_gauche = 255;
  }
  if (position < 3500)
  {
    speed_droite = 255;
    speed_gauche = (position) / 13.73;
  }
  Serial.print(" speed gauche = ");
  Serial.print(speed_gauche);
  Serial.print(" speed droit = ");
  Serial.println(speed_droite);

  turn(speed_gauche, speed_droite, 2);
}

void stop()
{
  Serial.println("stop");
  for (int j = 0; j < 4; j++)
  {
    tab[j].run(RELEASE);
    delay(10);
  }
}

void PIDLineFollower(int position, int goal)
{
  error = position - goal;
  int motorSpeed = Kp * error + Kd * (error - last_error);
  last_error = error;

  if (position > 3500)
  {
    speed_gauche = 255 - motorSpeed;
    speed_droite = 255;
  }

  if (position < 3500)
  {
    speed_gauche = 255;
    speed_droite = 255 - motorSpeed;
  }

  if (speed_gauche < 0)
  {
    speed_gauche = 0;
  }
  if (speed_droite < 0)
  {
    speed_droite = 0;
  }

  if (speed_gauche > 255)
  {
    speed_gauche = 255;
  }
  if (speed_droite > 255)
  {
    speed_droite = 255;
  }

  Serial.print(" speed gauche = ");
  Serial.print(speed_gauche);
  Serial.print(" speed droit = ");
  Serial.println(speed_droite);
}

void setup()
{
  Serial.begin(115200);
  // configure the sensors

  Serial.println("Booting");
  WiFi.mode(WIFI_STA);
  WiFi.begin(ssid, password);
  while (WiFi.waitForConnectResult() != WL_CONNECTED)
  {
    Serial.println("Connection Failed! Rebooting...");
    delay(5000);
    ESP.restart();
  }

  initOTA();

  Serial.println("Ready");
  Serial.print("IP address: ");
  Serial.println(WiFi.localIP());

  qtr.setTypeRC();
  qtr.setSensorPins((const uint8_t[]){21, 18, 12, 27, 14, 32, 15, 33}, SensorCount);
  calibrationLine();

  if (!AFMS.begin())
  { // create with the default frequency 1.6KHz
    // if (!AFMS.begin(1000)) {  // OR with a different frequency, say 1KHz
    Serial.println("Could not find Motor Shield. Check wiring.");
    while (1)
      ;
  }
  Serial.println("Motor Shield found.");
  // Set the speed to start, from 0 (off) to 255 (max speed)
  myMotor1->setSpeed(250);
  myMotor1->run(FORWARD);
  myMotor1->run(RELEASE);
  /*
  delay(1000);
  turn(250,250,1);
  delay(700);
  turn(250,0,1);
  delay(1700);
  turn(250, 250,1);
  delay(2800);
  turn(0,250,1);
  delay(1500);
  turn(250, 250,1);
  delay(900);
  stop();
  delay(3000);
  turn(250, 250,2);
  delay(2300);
  stop();
  delay(1000);
  turn(250,250,1);
  delay(700);
  turn(0,250,1);
  delay(1550);

  turn(250,250,1);
  delay(7000);

  delay(3000);
  turn(250, 250,2);
  delay(800);
  turn(250, 50,2);
  delay(2700);
  turn(250, 250,1);
  delay(7000);

  stop();
*/
}

void loop()
{
  // read calibrated sensor values and obtain a measure of the line position
  // from 0 to 5000 (for a white line, use readLineWhite() instead)
  ArduinoOTA.handle();
  int pos = getPositionLine();
  // getDistanceUS();
  LineFollower(pos);

  //PIDLineFollower(pos,3500);
  //delay(10);
}
