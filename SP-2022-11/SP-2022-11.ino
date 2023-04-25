#include <Arduino.h>
#if defined(ESP32)
#include <WiFi.h>
#include <FirebaseESP32.h>
#elif defined(ESP8266)
#include <ESP8266WiFi.h>
#include <FirebaseESP8266.h>
#elif defined(PICO_RP2040)
#include <WiFi.h>
#include <FirebaseESP8266.h>
#endif


#include <Wire.h>
#include "algorithm_by_RF.h"
#include "max30102.h"
#include <MAX3010x.h>
#include <math.h>
#include "Protocentral_MAX30205.h"


// Provide the token generation process info.
#include <addons/TokenHelper.h>

// Provide the RTDB payload printing info and other helper functions.
#include <addons/RTDBHelper.h>

/* 1. Define the WiFi credentials */
#define WIFI_SSID "Dome"
#define WIFI_PASSWORD "12345678"

// For the following credentials, see examples/Authentications/SignInAsUser/EmailPassword/EmailPassword.ino

/* 2. Define the API Key */
#define API_KEY "AIzaSyBNsUI9v7DUcJzlXrQ5YNJBmRs7WYsMmx0"

/* 3. Define the RTDB URL */
#define DATABASE_URL "fluted-arch-341414-default-rtdb.asia-southeast1.firebasedatabase.app" //<databaseName>.firebaseio.com or <databaseName>.<region>.firebasedatabase.app



FirebaseData fbdo;
FirebaseAuth auth;
FirebaseJsonArray arr; // you can add or set the array data
FirebaseConfig config;


unsigned long sendDataPrevMillis = 0;
unsigned long sendDataPrevMillis1 = 0;
bool signupOK = false;


#define DEBUG                                           // Uncomment for debug output to the Serial stream


// Interrupt pin
const byte oxiInt = 5;                                   // pin connected to MAX30102 INT                                     //Blinks with each data read
MAX30105 particleSensor;
MAX30205 tempSensor;

uint32_t aun_ir_buffer[BUFFER_SIZE];                      //infrared LED sensor data
uint32_t aun_red_buffer[BUFFER_SIZE];                     //red LED sensor data
float old_n_spo2;                                         // Previous SPO2 value
uint8_t uch_dummy,k;
bool isFingerPlaced = false;
unsigned long timer1_start_time = 0;
unsigned long timer2_start_time = 0;

void setup() {
  Serial.begin(115200);

   WiFi.begin(WIFI_SSID, WIFI_PASSWORD);
   Serial.print("Connecting to Wi-Fi");
  while (WiFi.status() != WL_CONNECTED)
  {
    Serial.print(".");
    delay(300);
  }
  Serial.println();
  Serial.print("Connected with IP: ");
  Serial.println(WiFi.localIP());
  Serial.println();

  Serial.printf("Firebase Client v%s\n\n", FIREBASE_CLIENT_VERSION);

    /* Assign the api key (required) */
  config.api_key = API_KEY;
  /* Assign the RTDB URL (required) */
  config.database_url = DATABASE_URL;

  if(Firebase.signUp(&config, &auth, "","")){
    Serial.println("SignUp OK");
    signupOK = true;
  }else{
    Serial.printf("%s\n", config.signer.signupError.message.c_str());
  }


  config.token_status_callback = tokenStatusCallback;
  Firebase.begin(&config, &auth);
  Firebase.reconnectWiFi(true);
  
  Wire.setClock(400000);                                  // Set I2C speed to 400kHz
  
  pinMode(oxiInt, INPUT);                                 //pin D10 connects to the interrupt output pin of the MAX30102
  Wire.begin();
  tempSensor.begin();  
  particleSensor.begin();         // Use default I2C port, 400kHz speed
  particleSensor.setSamplingRate(particleSensor.SAMPLING_RATE_100SPS);
  while(!tempSensor.scanAvailableSensors()){
    Serial.println("Couldn't find the temperature sensor, please connect the sensor." );
    delay(30000);
  }
  
  maxim_max30102_reset();                                 //resets the MAX30102
  delay(1000);
  maxim_max30102_read_reg(REG_INTR_STATUS_1,&uch_dummy);  //Reads/clears the interrupt status register
  maxim_max30102_init();                                  //initialize the MAX30102
  old_n_spo2=0.0;

  uint8_t revID;
  uint8_t partID;
  
  maxim_max30102_read_reg(0xFE, &revID);
  maxim_max30102_read_reg(0xFF, &partID);


//  if ( Firebase.deleteNode(fbdo,"Red/value")) {
//    Serial.println ("Successfully deleted node at:");
//    Serial.println ("Red");
//   }
//   else{
//    Serial.println ("Error deleteing data at:");
//    Serial.println ("Red");
//    Serial.println(fbdo.errorReason());
//  } 
//    delay(3000);
  
}

//Continuously taking samples from MAX30102.  Heart rate and SpO2 are calculated every ST seconds
void loop() {
  
  processIR();
  processHRandSPO2();
  delay(10000);

}

void processIR(){
  auto sample3 = particleSensor.readSample(100);
  long irValue1 = sample3.ir; 

  if (irValue1 > 600000){

      for(int i=0;i<1400;i++){
        auto sample1 = particleSensor.readSample(100);
        arr.add(sample1.ir);
        Serial.println(sample1.ir);
      }
      Serial.println(ESP.getFreeHeap());
      delay(10000);
      
       if (Firebase.ready() && (millis() - sendDataPrevMillis1 > 15000 || sendDataPrevMillis1 == 0))
      {
        sendDataPrevMillis1 = millis();  

        if (Firebase.setArray(fbdo, "Red/value", arr)) //or Firebase.set(fbdo, "path/to/node", arr)
        {
          Serial.println("set array ok");
        }else{
          Serial.println("set array failed");
          Serial.println(fbdo.errorReason());
        }
      }

      arr.clear();  
  }
}

void processHRandSPO2(){
  unsigned long timer2_start_time = millis();
  unsigned long elapsed_time_timer2 = 0; // calculate elapsed time for timer 2
  while(elapsed_time_timer2< 65000){
    auto sample = particleSensor.readSample(50);
    long irValue = sample.ir;                  // Reading the IR value it will permit us to know if there's a finger on the sensor or not
    if (irValue > 700000){
      if(isFingerPlaced == false){
        isFingerPlaced = true;
    
      }
      float n_spo2,ratio,correl;                            //SPO2 value
      int8_t ch_spo2_valid;                                 //indicator to show if the SPO2 calculation is valid
      int32_t n_heart_rate;                                 //heart rate value
      int8_t  ch_hr_valid;                                  //indicator to show if the heart rate calculation is valid
      int32_t i;
    
         
      //buffer length of BUFFER_SIZE stores ST seconds of samples running at FS sps
      //read BUFFER_SIZE samples, and determine the signal range
      for(i=0;i<BUFFER_SIZE;i++)
      {
        while(digitalRead(oxiInt)==1){                      //wait until the interrupt pin asserts
           yield();
        }
        
        //IMPORTANT:
        //IR and LED are swapped here for MH-ET MAX30102. Check your vendor for MAX30102
        //and use this or the commented-out function call.
        maxim_max30102_read_fifo((aun_ir_buffer+i), (aun_red_buffer+i));
        //maxim_max30102_read_fifo((aun_red_buffer+i), (aun_ir_buffer+i));
      }
    
      //calculate heart rate and SpO2 after BUFFER_SIZE samples (ST seconds of samples) using Robert's method
      rf_heart_rate_and_oxygen_saturation(aun_ir_buffer, BUFFER_SIZE, aun_red_buffer, &n_spo2, &ch_spo2_valid, 
      &n_heart_rate, &ch_hr_valid, &ratio, &correl); 
   
      float temp = tempSensor.getTemperature(); // read temperature for every 100ms;
      float hr_cali = (0.7504*n_heart_rate)+21.255;
      float spo2_cali = (((n_spo2-95.59)*4)/4.37)+96;
      if (Firebase.ready() && (millis() - sendDataPrevMillis > 15000 || sendDataPrevMillis == 0))
        {
          sendDataPrevMillis = millis();
          delay(1000);
           
           if(Firebase.RTDB.setString(&fbdo, "Sensor/bodytemp/data", ceil(temp))){
            Serial.println("Successfully save Temp");
           }else{
            Serial.println("Failed"+fbdo.errorReason());
           }
        
          if(ceil(n_spo2) != -999){
            
            if(Firebase.RTDB.setString(&fbdo, "Sensor/spo2/data", ceil(spo2_cali))){
              Serial.println("Successfully save SPO2");
            }else{
              Serial.println("Failed"+fbdo.errorReason());
            }
            
          }
          
         if(ceil(n_heart_rate)!= -999){
            if(Firebase.RTDB.setString(&fbdo, "Sensor/hr/data", ceil(hr_cali))){
              Serial.println("Successfully save HR");
            }else{
              Serial.println("Failed"+fbdo.errorReason());
            }
  
         }
     }
    }else{
    
      isFingerPlaced = false;
      Serial.println("No finger");
      
    }
      unsigned long current_time1 = millis(); // get the current time
      elapsed_time_timer2 = current_time1 - timer2_start_time;
  }
  
    timer2_start_time = millis(); //
  
}
