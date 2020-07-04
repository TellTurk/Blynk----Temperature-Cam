//you have to comment out the line
// #include <Adafruit_Sensor.h> in the library <Adafruit_BME280.h>
#include <Blynk.h>
// #define BLYNK_PRINT Serial
#include <Wire.h>
// #include <DHT.h> // If you want to use a DHT sensor, uncomment this line
#include <SPI.h>
#include "Adafruit_Sensor.h">
#include "Adafruit_BME280.h" //If you use a DHT sensor, you can comment out this line
#include <math.h>
#include "src/OV2640.h"
#include <WiFi.h>
#include <WebServer.h>
#include <WiFiClient.h>
#include <BlynkSimpleEsp32.h>

// Select camera model
//#define CAMERA_MODEL_WROVER_KIT
//#define CAMERA_MODEL_ESP_EYE
//#define CAMERA_MODEL_M5STACK_PSRAM
//#define CAMERA_MODEL_M5STACK_WIDE
#define CAMERA_MODEL_AI_THINKER // This is the camera model for the ESP-32-Cam 
#define I2C_SDA 3
#define I2C_SCL 1


#include "camera_pins.h"

// Authentication token from Blynk (provided when creating app, otherwise click the nut icon)
char auth[] = "REDACTED"; 

//Replace with your wifi name and password, exactly correct. If your wifi has a landing page (e.g. like trains and hotels that require you to sign in, this code will not work - I mention this for the one person who will go mad realizing this)
#define SSID1 "REDACTED"
#define PWD1 "REDACTED"

/* DELETE THE slash star on either side of this block of code to uncomment this section for water temperature sensor
// For measuring the temperature of water, we use a thermistor, this configuration is required. If you use a sensor/resistor with different characteristics, this will have to be modified.
// resistance at 25 degrees C
#define THERMISTORNOMINAL 10000      
// temp. for nominal resistance (almost always 25 C)
#define TEMPERATURENOMINAL 25   
// how many samples to take and average, more takes longer
// but is more 'smooth'
// The beta coefficient of the thermistor (usually 3000-4000)
#define BCOEFFICIENT 3950
// the value of the 'other' resistor
#define SERIESRESISTOR 10000    
const double VCC = 3.3;             // NodeMCU on board 3.3v vcc
const double R2 = 10000;            // 10k ohm series resistor
const double adc_resolution = 1023; // 10-bit adc
//This is the number of samples that will be taken for the water temperature sensor (to get an average and increase accuracy)
#define NUMSAMPLES 200
int samples[NUMSAMPLES];
*/
OV2640 cam;
TwoWire I2CBME = TwoWire(0);
Adafruit_BME280 bme; // I2C
 

/*
Adafruit_BME280 bme(BME_CS); // hardware SPI
Adafruit_BME280 bme(BME_CS, BME_MOSI, BME_MISO, BME_SCK); // software SPI
#define DHTTYPE DHT22   // DHT 22, AM2302, AM2321
DHT dht(DHTPIN, DHTTYPE);
#define BME_SCK 13
#define BME_MISO 12
#define BME_MOSI 11
#define BME_CS 10
 BME280 GND --> NodeMCU GND
 BME280 3.3V --> NodeMCU 3V3
 BME280 SDA --> NodeMCU D2
 BME280 SCL --> NodeMCU D1 */

BlynkTimer timer;

WebServer server(80);

const char HEADER[] = "HTTP/1.1 200 OK\r\n" \
                      "Access-Control-Allow-Origin: *\r\n" \
                      "Content-Type: multipart/x-mixed-replace; boundary=123456789000000000000987654321\r\n";
const char BOUNDARY[] = "\r\n--123456789000000000000987654321\r\n";
const char CTNTTYPE[] = "Content-Type: image/jpeg\r\nContent-Length: ";
const int hdrLen = strlen(HEADER);
const int bdrLen = strlen(BOUNDARY);
const int cntLen = strlen(CTNTTYPE);

void handle_jpg_stream(void)
{
  char buf[32];
  int s;

  WiFiClient client = server.client();

  client.write(HEADER, hdrLen);
  client.write(BOUNDARY, bdrLen);

  while (true)
  {
    if (!client.connected()) break;
    cam.run();
    s = cam.getSize();
    client.write(CTNTTYPE, cntLen);
    sprintf( buf, "%d\r\n\r\n", s );
    client.write(buf, strlen(buf));
    client.write((char *)cam.getfb(), s);
    client.write(BOUNDARY, bdrLen);
  }
}

const char JHEADER[] = "HTTP/1.1 200 OK\r\n" \
                       "Content-disposition: inline; filename=capture.jpg\r\n" \
                       "Content-type: image/jpeg\r\n\r\n";
const int jhdLen = strlen(JHEADER);

void handle_jpg(void)
{
  WiFiClient client = server.client();

  cam.run();
  if (!client.connected()) return;

  client.write(JHEADER, jhdLen);
  client.write((char *)cam.getfb(), cam.getSize());
}

void handleNotFound()
{
  String message = "Server is running!\n\n";
  message += "URI: ";
  message += server.uri();
  message += "\nMethod: ";
  message += (server.method() == HTTP_GET) ? "GET" : "POST";
  message += "\nArguments: ";
  message += server.args();
  message += "\n";
  server.send(200, "text / plain", message);
}
void sendSensor()
{
//  float h = dht.readHumidity();
//  float t = dht.readTemperature(true); 

  float t2inC = bme.readTemperature();
  float t2 = (t2inC * 9 / 5) + 32; // convert to F from C
  float h2 = bme.readHumidity(); //
  float p = (bme.readPressure() / 100.0F);
//calculate VPD 
  float svp = 610.7 * (pow(10, (7.5*t2inC/(237.3+t2inC))));  
  float VPD = (((100 - h2)/100)*svp)/1000; 
  if (isnan(p) || isnan(t2) || isnan(h2) || isnan(t2inC)) {
 //   Serial.println("Failed to read from DHT sensor!");
    return; 
  }
  /*
  uint8_t i;
  float average, TempinFzxs;
   for (i=0; i< NUMSAMPLES; i++) {
    samples[i] = analogRead(A0); //hear we read the water sensor
     delay(50);
   }
     average = 0;
  for (i=0; i< NUMSAMPLES; i++) {
     average += samples[i];
  }
  average /= NUMSAMPLES;

  double Vout, Rth, temperature, adc_value;
  float TempinF; 
  adc_value = average;
 // adc_value = analogRead(A0);
  Vout = (adc_value * VCC) / adc_resolution;
  Rth = (VCC * R2 / Vout) - R2;

//  Steinhart-Hart Thermistor Equation:
//  Temperature in Kelvin = 1 / (A + B[ln(R)] + C[ln(R)]^3)
//  where A = 0.001129148, B = 0.000234125 and C = 8.76741*10^-8  
//  temperature = (1 / (A + (B * log(Rth)) + (C * pow((log(Rth)),3))));   // Temperature in kelvin

// temperature = temperature - 273.15;  // Temperature in degree celsius
// TempinF = (temperature * 9.0)/ 5.0 + 32.0; 

*/


//  Blynk.virtualWrite(V5, t);
//  Blynk.virtualWrite(V6, h);
  Blynk.virtualWrite(V7, String(t2,2));
  Blynk.virtualWrite(V8, h2);
  Blynk.virtualWrite(V9, p);
  Blynk.virtualWrite(V10, VPD);
//  Blynk.virtualWrite(V13, String(TempinF, 2));
}



void setup()
{

 // Serial.begin(115200);

  Blynk.begin(auth, SSID1, PWD1);
  // You can also specify server:
  //Blynk.begin(auth, SSID1, PWD1, "blynk-cloud.com", 8442);
  //Blynk.begin(auth, SSID1, PWD1, IPAddress(192,168,1,100), 8442);
    I2CBME.begin(I2C_SDA, I2C_SCL, 100000); 
    bme.begin(0x76, &I2CBME);
//    dht.begin();
    // Setup a function to be called every second
    timer.setInterval(6000L, sendSensor); // Frequency of updates
  //while (!Serial);            //wait for serial connection.

  camera_config_t config;
  config.ledc_channel = LEDC_CHANNEL_0;
  config.ledc_timer = LEDC_TIMER_0;
  config.pin_d0 = Y2_GPIO_NUM;
  config.pin_d1 = Y3_GPIO_NUM;
  config.pin_d2 = Y4_GPIO_NUM;
  config.pin_d3 = Y5_GPIO_NUM;
  config.pin_d4 = Y6_GPIO_NUM;
  config.pin_d5 = Y7_GPIO_NUM;
  config.pin_d6 = Y8_GPIO_NUM;
  config.pin_d7 = Y9_GPIO_NUM;
  config.pin_xclk = XCLK_GPIO_NUM;
  config.pin_pclk = PCLK_GPIO_NUM;
  config.pin_vsync = VSYNC_GPIO_NUM;
  config.pin_href = HREF_GPIO_NUM;
  config.pin_sscb_sda = SIOD_GPIO_NUM;
  config.pin_sscb_scl = SIOC_GPIO_NUM;
  config.pin_pwdn = PWDN_GPIO_NUM;
  config.pin_reset = RESET_GPIO_NUM;
  config.xclk_freq_hz = 20000000;
  config.pixel_format = PIXFORMAT_JPEG;

  // Frame parameters
    config.frame_size = FRAMESIZE_UXGA;
 // config.frame_size = FRAMESIZE_QVGA;
  config.jpeg_quality = 3;
  config.fb_count = 2;

#if defined(CAMERA_MODEL_ESP_EYE)
  pinMode(13, INPUT_PULLUP);
  pinMode(14, INPUT_PULLUP);
#endif

  cam.init(config);

  IPAddress ip;

  WiFi.mode(WIFI_STA);
  WiFi.begin(SSID1, PWD1);
  while (WiFi.status() != WL_CONNECTED)
  {
    delay(500);
 //   Serial.print(F("."));
  }
  ip = WiFi.localIP();
 // Serial.println(F("WiFi connected"));
 // Serial.println("");
 // Serial.println(ip);
//  Serial.print("Stream Link: http://");
//  Serial.print(ip);
//Serial.println("/mjpeg/1");
  server.on("/mjpeg/1", HTTP_GET, handle_jpg_stream);
  server.on("/jpg", HTTP_GET, handle_jpg);
  server.onNotFound(handleNotFound);
  server.begin();
}

void loop()
{
  Blynk.run();
  server.handleClient();
  timer.run();
}
