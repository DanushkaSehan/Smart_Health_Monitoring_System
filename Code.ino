
//OLED and max30102 configuration --------------
#include <Adafruit_GFX.h>        // OLED libraries
#include <Adafruit_SSD1306.h>
#include <Wire.h>
#include "MAX30105.h"           // MAX3010x library
#include "heartRate.h"          // Heart rate calculating algorithm

#define BLYNK_TEMPLATE_ID "------"    //ID
#define BLYNK_TEMPLATE_NAME "--"
#define BLYNK_AUTH_TOKEN "--"
#define BLYNK_PRINT Serial

#include <WiFiClient.h>
#include <BlynkSimpleEsp32.h>



//accelometer configuration ---------------------
#include <Adafruit_Sensor.h> 
#include <Adafruit_ADXL345_U.h>



#include <WiFi.h>
#include "time.h"



char auth[] = BLYNK_AUTH_TOKEN;
char ssid[] = "DANUSHKA";
char pass[] = "aaaaaaaa";



const char* ntpServer = "pool.ntp.org";
const long  gmtOffset_sec = 19800;
const int   daylightOffset_sec = 0;


int pin2 = 4;
// Time 1
int startTime3;
int endTime3;
int day3;
// time 2
int startTime4;
int endTime4;
int day4;
// Time 3
int startTime5;
int endTime5;
int day5;

int startTime6;
int endTime6;

#define SCREEN_WIDTH  128 // OLED display width, in pixels
#define SCREEN_HEIGHT 32 // OLED display height, in pixels
#define OLED_RESET    -1 // Reset pin # (or -1 if sharing Arduino reset pin)

Adafruit_SSD1306 display(SCREEN_WIDTH, SCREEN_HEIGHT, &Wire, OLED_RESET);  // Declaring the display name (display)



Adafruit_ADXL345_Unified accel = Adafruit_ADXL345_Unified();

MAX30105 particleSensor;

const byte RATE_SIZE = 4; // Increase this for more averaging. 4 is good.
byte rates[RATE_SIZE]; // Array of heart rates
byte rateSpot = 0;
long lastBeat = 0; // Time at which the last beat occurred
float beatsPerMinute;
int beatAvg;
int spo2 = 0;

bool motionDetectionEnabled = false;



static const unsigned char PROGMEM  logo2_bmp[] =
{
  0x03, 0xC0, 0xF0, 0x06, 0x71, 0x8C, 0x0C, 0x1B, 0x06, 0x18, 0x0E,  0x02, 0x10, 0x0C, 0x03, 0x10,              // Logo2 and Logo3 are two bmp pictures that display on the OLED if called
  0x04, 0x01, 0x10, 0x04, 0x01, 0x10, 0x40,  0x01, 0x10, 0x40, 0x01, 0x10, 0xC0, 0x03, 0x08, 0x88,
  0x02, 0x08, 0xB8, 0x04,  0xFF, 0x37, 0x08, 0x01, 0x30, 0x18, 0x01, 0x90, 0x30, 0x00, 0xC0, 0x60,
  0x00,  0x60, 0xC0, 0x00, 0x31, 0x80, 0x00, 0x1B, 0x00, 0x00, 0x0E, 0x00, 0x00, 0x04, 0x00,  
};

static const unsigned char PROGMEM logo3_bmp[] =
{
  0x01, 0xF0, 0x0F,  0x80, 0x06, 0x1C, 0x38, 0x60, 0x18, 0x06, 0x60, 0x18, 0x10, 0x01, 0x80, 0x08,
  0x20,  0x01, 0x80, 0x04, 0x40, 0x00, 0x00, 0x02, 0x40, 0x00, 0x00, 0x02, 0xC0, 0x00, 0x08,  0x03,
  0x80, 0x00, 0x08, 0x01, 0x80, 0x00, 0x18, 0x01, 0x80, 0x00, 0x1C, 0x01,  0x80, 0x00, 0x14, 0x00,
  0x80, 0x00, 0x14, 0x00, 0x80, 0x00, 0x14, 0x00, 0x40,  0x10, 0x12, 0x00, 0x40, 0x10, 0x12, 0x00,
  0x7E, 0x1F, 0x23, 0xFE, 0x03, 0x31,  0xA0, 0x04, 0x01, 0xA0, 0xA0, 0x0C, 0x00, 0xA0, 0xA0, 0x08,
  0x00, 0x60, 0xE0,  0x10, 0x00, 0x20, 0x60, 0x20, 0x06, 0x00, 0x40, 0x60, 0x03, 0x00, 0x40, 0xC0,
  0x01,  0x80, 0x01, 0x80, 0x00, 0xC0, 0x03, 0x00, 0x00, 0x60, 0x06, 0x00, 0x00, 0x30, 0x0C,  0x00,
  0x00, 0x08, 0x10, 0x00, 0x00, 0x06, 0x60, 0x00, 0x00, 0x03, 0xC0, 0x00,  0x00, 0x01, 0x80, 0x00  
};





void setup(){
  Serial.begin(115200);
  display.begin(SSD1306_SWITCHCAPVCC,  0x3C); // Start the OLED display
  display.display();

  // Connect to Wi-Fi
  Serial.print("Connecting to ");
  Serial.println(ssid);
  WiFi.begin(ssid, pass);
  while (WiFi.status() != WL_CONNECTED) {
    delay(1000);
    Serial.println("Connecting to WiFi...");
  }
  Serial.println("Connected to WiFi");

  // Connect to Blynk
  Blynk.begin(auth, ssid, pass);
  while (!Blynk.connected()) {
    Serial.println("Connecting to Blynk...");
    delay(1000);
  }
  
  // Init and get the time
  configTime(gmtOffset_sec, daylightOffset_sec, ntpServer);
  //printLocalTime();

  pinMode(pin2, OUTPUT);
  digitalWrite(pin2, LOW);

    if(!accel.begin())
   {
      Serial.println("No valid sensor found");
      while(1);
   }




  
  delay(3000);
  // Initialize sensor
  particleSensor.begin(Wire, I2C_SPEED_FAST); // Use default I2C port, 400kHz speed
  particleSensor.setup(); // Configure sensor with default settings
  particleSensor.setPulseAmplitudeRed(0x0A); // Turn Red LED to low to indicate sensor is running
  
  

Blynk.virtualWrite(V2, 0);
}

//accelometer on/off 
BLYNK_WRITE(V2) {
  int switchState = param.asInt(); // Get the value from the switch button widget
  if (switchState == 1) {
    motionDetectionEnabled = true;
    Serial.println("Motion detection enabled");
  } else {
    motionDetectionEnabled = false;
    Serial.println("Motion detection disabled");
  }
}




BLYNK_WRITE(V6){
 startTime3 = param[0].asInt();
 endTime3 = param[1].asInt();
 day3 = param[3].asInt();
 if(startTime3 == 0 && endTime3 == 0)
 {
  startTime3 = 999999;
  endTime3 = 999999;
 }
 Serial.println(startTime3);
   Serial.println(endTime3);
}

BLYNK_WRITE(V7){
 startTime4 = param[0].asInt();
 endTime4 = param[1].asInt();
 day4 = param[3].asInt();
 if(startTime4 == 0 && endTime4 == 0)
 {
  startTime4 = 999999;
  endTime4 = 999999;
 }
 Serial.println(startTime4);
   Serial.println(endTime4);
}
BLYNK_WRITE(V8){
 startTime5 = param[0].asInt();
 endTime5 = param[1].asInt();
 day5 = param[3].asInt();
 if(startTime5 == 0 && endTime5 == 0)
 {
  startTime5 = 999999;
  endTime5 = 999999;
 }
 Serial.println(startTime5);
   Serial.println(endTime5);
}
BLYNK_WRITE(V9){
 startTime6 = param[0].asInt();
 endTime6 = param[1].asInt();
 if(startTime6 == 0 && endTime6 == 0)
 {
  startTime6 = 999999;
  endTime6 = 999999;
 }
 Serial.println(startTime6);
   Serial.println(endTime6);
}

void loop(){
  
  printLocalTime();
  heart_measure();
  motion_detection();
  Blynk.run();
}

void motion_detection(){
    if (motionDetectionEnabled) {
    sensors_event_t event; 
    accel.getEvent(&event);

    Serial.println("X: "); Serial.print(event.acceleration.x); Serial.print("  ");
    Serial.println("Y: "); Serial.print(event.acceleration.y); Serial.print("  ");
    Serial.println("Z: "); Serial.print(event.acceleration.z); Serial.print("  ");

    if(event.acceleration.x > 3 || event.acceleration.y > 3 || event.acceleration.z > 3 ||
       event.acceleration.x < -14 || event.acceleration.y < -14 || event.acceleration.z < -14) {
      
      Blynk.logEvent("motion_detected");
      // Add any other actions you want to perform when motion is detected
      delay(500); // Delay to avoid repeated detection in a short time
    }

    //delay(500); // Adjust the delay according to your needs for motion detection sensitivity and performance
  }
  }

void heart_measure(){
    long irValue = particleSensor.getIR(); // Reading the IR value to check if there's a finger on the sensor or not, also detecting a heartbeat
   


  if (irValue > 7000) { // If a finger is detected

   float R = particleSensor.getRed() / (float)particleSensor.getIR();;
    spo2 = 104 - 17 * R; // Simple estimation for SpO2 calculation
    
    display.clearDisplay(); // Clear the display
    display.drawBitmap(5, 5, logo2_bmp, 24, 21, WHITE); // Draw the first bmp picture (little heart)
    display.setTextSize(2); // Near it, display the average BPM
    display.setTextColor(WHITE);  
    display.setCursor(50, 0); 
    display.setTextSize(1);               
    display.println("BPM");             
    display.setCursor(50, 18);
    display.setTextSize(2);                
    display.println(beatAvg);

    display.setCursor(85, 0); 
    display.setTextSize(1);               
    display.println("SOP2");             
    display.setCursor(95, 18);
    display.setTextSize(2);                
    display.println(spo2);    
    display.display();
    
    if (checkForBeat(irValue) == true) { // If a heart beat is detected
      display.clearDisplay(); // Clear the display
      display.drawBitmap(0, 0, logo3_bmp, 32, 32, WHITE); // Draw the second picture (bigger heart)
      display.setTextSize(2); // And still display the average BPM
      display.setTextColor(WHITE);             
      display.setCursor(50, 0); 
      display.setTextSize(1);               
      display.println("BPM");             
      display.setCursor(50, 18);
      display.setTextSize(2);                 
      display.println(beatAvg);

      if(beatAvg>60){
        Blynk.logEvent("bpm_is_high");
      }
      delay(200);

      display.setCursor(85, 0); 
      display.setTextSize(1);               
      display.println("SOP2");             
      display.setCursor(95, 18);
      display.setTextSize(2);                
      display.println(spo2); 
      display.display();  
      delay(100);
      

      //avgbpm send to the BLYNK
      Blynk.virtualWrite(V0, beatAvg);
      Blynk.virtualWrite(V1, spo2);
      

      // We sensed a beat!
      long delta = millis() - lastBeat; // Measure duration between two beats
      lastBeat = millis();
      beatsPerMinute = 60 / (delta / 1000.0); // Calculating the BPM
      if (beatsPerMinute < 255 && beatsPerMinute > 20) { // To calculate the average, we store some values (4) then do some math to calculate the average
        rates[rateSpot++] = (byte)beatsPerMinute; // Store this reading in the array
        rateSpot %= RATE_SIZE; // Wrap variable
        // Take average of readings
        beatAvg = 0;
        for (byte x = 0; x < RATE_SIZE; x++)
          beatAvg += rates[x];
        beatAvg /= RATE_SIZE;
      }
      
    }
  }
  if (irValue < 7000) { // If no finger is detected, it informs the user and puts the average BPM to 0 or it will be stored for the next measure
    beatAvg = 0;
    spo2 = 0;
    Blynk.virtualWrite(V0, beatAvg);
    Blynk.virtualWrite(V1, spo2);
    display.clearDisplay();
    display.setTextSize(1);                    
    display.setTextColor(WHITE);             
    display.setCursor(30, 5);                
    display.println("Please Place "); 
    display.setCursor(30, 15);
    display.println("your finger ");  
    display.display();
    
  } 
}

void printLocalTime(){
  struct tm timeinfo;
  if(!getLocalTime(&timeinfo)){
    Serial.println("Failed to obtain time");
    return;
  }

  int HH = timeinfo.tm_hour;
  int MM = timeinfo.tm_min;
  
  int server_time = 3600*HH + 60*MM;
  // Serial.println("server_time - ", server_time);


  if (((server_time >= startTime3 && server_time < endTime3)) || ((server_time >= startTime4 && server_time < endTime4))  || ((server_time >= startTime5 && server_time < endTime5))  ||   ((server_time >= startTime6 && server_time < endTime6) )) {
    //digitalWrite(pin2, HIGH); // Turn on the buzzer
    tone(pin2, 3000, 5);
    Serial.println("Buzzer ON");
  } else {
    //digitalWrite(pin2, LOW); // Turn off the buzzer
    noTone(pin2);
    Serial.println("Buzzer OFF");
  }
}
