#define ENABLE_USER_AUTH
#define ENABLE_FIRESTORE

#include <Arduino.h>
#include <ArduinoJson.h>
#include <FirebaseClient.h>
#include <LiquidCrystal_I2C.h>
#include <WiFi.h>
#include <WiFiClientSecure.h>
#include <Wire.h> 

// Network and Firebase credentials
#define WIFI_SSID ""
#define WIFI_PASSWORD ""
#define Web_API_KEY ""
#define USER_EMAIL ""
#define USER_PASSWORD ""
#define FIREBASE_PROJECT_ID ""

#define WATERFLOW_SENSOR_PIN 23
#define VALVE_CONTROLLER_PIN 2

#define VOLUM_COMP_INTERVAL 500
#define VALVE_SYNC_INTERVAL 2000
#define VOLUM_UPDT_INTERVAL 5000

// Authentication
UserAuth userAuth(Web_API_KEY, USER_EMAIL, USER_PASSWORD);

// Firebase components
FirebaseApp appUser, appService;
WiFiClientSecure sslClientUser;
using AsyncClient = AsyncClientClass;
AsyncClient userClient(sslClientUser);

Firestore::Documents docs;
AsyncResult firestoreResult;

LiquidCrystal_I2C lcd(0x27,16,2);  // set the LCD address to 0x27 for a 16 chars and 2 line display

String DEVICE_NAME = "test_device_1";

volatile int pulse;

double totalVolume, lastTotalVolume;
bool valveStatus, flowInd;
int maxFlow = 0;

unsigned long currTime;
unsigned long lastValveSyncTime, lastVolUpdateTime, lastVolCompTime; 
unsigned long startFlowTime, lastLeakCheckTime;
uint32_t ntpTime;

// User function
void processData(AsyncResult &aResult);

void setup() {

  Serial.begin(115200);
  delay(10);
  
  connectWiFi();

  lcd.init();
  lcd.backlight();

  pinMode(WATERFLOW_SENSOR_PIN, INPUT);
  pinMode(VALVE_CONTROLLER_PIN, OUTPUT);
  attachInterrupt(digitalPinToInterrupt(WATERFLOW_SENSOR_PIN), increasePulse, RISING);

  ntpTime = get_ntp_time();

  // Initialize Firebase
  sslClientUser.setInsecure();
  appService.setTime(ntpTime);

  initializeApp(userClient, appUser, getAuth(userAuth), 120 * 1000, processData); //Initialze and wait
  appUser.getApp<Firestore::Documents>(docs);
  
  syncDevice();
  initFields();
}

void increasePulse() {

  pulse++;

}

void connectWiFi() {
  //Connect to Wi-Fi
  WiFi.mode(WIFI_STA);
  WiFi.begin(WIFI_SSID, WIFI_PASSWORD);

  Serial.print("Connecting to WiFi..");  
  
  while (WiFi.status() != WL_CONNECTED) {
    Serial.print(".");
    delay(300);
  }

  Serial.println("\nWiFi connected!");
}

void processData(AsyncResult &aResult){
    // Exits when no result is available when calling from the loop.
    if (!aResult.isResult())
        return;

    if (aResult.isEvent()){
        Firebase.printf("Event task: %s, msg: %s, code: %d\n", aResult.uid().c_str(), aResult.eventLog().message().c_str(), aResult.eventLog().code());
    }

    if (aResult.isDebug()){
        Firebase.printf("Debug task: %s, msg: %s\n", aResult.uid().c_str(), aResult.debug().c_str());
    }

    if (aResult.isError()){
        Firebase.printf("Error task: %s, msg: %s, code: %d\n", aResult.uid().c_str(), aResult.error().message().c_str(), aResult.error().code());
    }

    if (aResult.available()){
      //  Firebase.printf("task: %s, payload: %s\n", aResult.uid().c_str(), aResult.c_str());
    }
}


void syncDevice(){
  
  //Retreive device info

  String payload = docs.get(userClient, Firestore::Parent(FIREBASE_PROJECT_ID), "devices/" + DEVICE_NAME, GetDocumentOptions());
  
  if (userClient.lastError().code() == 0){

      JsonDocument doc;
      deserializeJson(doc, payload);

      valveStatus = doc["fields"]["valve_status"]["booleanValue"];

      totalVolume = doc["fields"]["total_consumption"]["doubleValue"];
      lastTotalVolume = totalVolume;

      int maxFlowSec = doc["fields"]["max_flow"]["integerValue"];
      maxFlow = maxFlowSec * 1000; //convert into milliseconds;

      //Display water consumption
      lcd.setCursor(0,0);
      lcd.print("Mtr: " + String(totalVolume, 2) + "    ");

      //Display valve status
      lcd.setCursor(0,1);

      if (valveStatus == true){

        digitalWrite(VALVE_CONTROLLER_PIN, LOW);
        lcd.print("Valve: OPEN   ");
        Serial.println("valveStatus is TRUE");      
      
      }
      else{

        digitalWrite(VALVE_CONTROLLER_PIN, HIGH);
        lcd.print("Valve: CLOSED ");
        Serial.println("valveStatus is FALSE");      
      
      }

  }else
      Firebase.printf("Error retrieving device info, msg: %s, code: %d\n", userClient.lastError().message().c_str(), userClient.lastError().code());
}

void initFields(){

  currTime = millis();
  
  lastVolCompTime = currTime;
  lastValveSyncTime = currTime;
  lastVolUpdateTime = currTime;
  lastLeakCheckTime = currTime;

}

void loop() {

  if (WiFi.status() != WL_CONNECTED) {
    connectWiFi();
  }
  // Maintain the authentication and async tasks of firebase
  appUser.loop();

  // Get current time
  currTime = millis();

  //Calculate and accumulate the water volume every VOLUM_COMP_INTERVAL milliseconds
  //Also check for water leak.
  if(currTime - lastVolCompTime > VOLUM_COMP_INTERVAL) {

    lastVolCompTime = currTime;

    if (pulse > 0){

      totalVolume += (pulse * 0.000333333);
      pulse = 0;

      flowInd = true;

      //Display the water volume consumed
      lcd.setCursor(0,0);
      lcd.print("Mtr: " + String(totalVolume, 2) + "    ");
    }

    checkWaterLeak();

  }

  //Sync the valve status every VALVE_SYNC_INTERVAL milliseconds
  if (currTime - lastValveSyncTime > VALVE_SYNC_INTERVAL){

    lastValveSyncTime = currTime;
    
    if (appUser.ready()) //Apply sync when firebase app is ready
      syncValveAndMaxFlow();
  }

  //Update the volume in cloud database every VOLUM_UPDT_INTERVAL seconds.
  if (currTime - lastVolUpdateTime  > VOLUM_UPDT_INTERVAL) {

    lastVolUpdateTime = currTime;
    
    if (lastTotalVolume != totalVolume){

      lastTotalVolume = totalVolume;

      if (appUser.ready()) { //Apply update whe firebase app is ready
        updateVolConsumption();
      }
    }
  }
}

void syncValveAndMaxFlow(){
  //Retreive device info
  String payload = docs.get(userClient, Firestore::Parent(FIREBASE_PROJECT_ID), "devices/" + DEVICE_NAME, GetDocumentOptions());

  if (userClient.lastError().code() == 0){
    
      JsonDocument doc;
      deserializeJson(doc, payload);

      valveStatus = doc["fields"]["valve_status"]["booleanValue"];

      int maxFlowSec = doc["fields"]["max_flow"]["integerValue"];
      maxFlow = maxFlowSec * 1000; //convert into milliseconds;

      //Display valve status
      lcd.setCursor(0,1);

      if (valveStatus == true){

        digitalWrite(VALVE_CONTROLLER_PIN, LOW);
        lcd.print("Valve: OPEN   ");
        Serial.println("valveStatus is TRUE");      
      
      }
      else{

        digitalWrite(VALVE_CONTROLLER_PIN, HIGH);
        lcd.print("Valve: CLOSED");
        Serial.println("valveStatus is FALSE");      
      
      }

  }else
      Firebase.printf("Error retrieving valve_status, msg: %s, code: %d\n", userClient.lastError().message().c_str(), userClient.lastError().code());
}

//Update the database with the new total water volume/consumption
void updateVolConsumption(){  

  Values::DoubleValue doubleV(totalVolume);
  Document<Values::Value> doc("total_consumption", Values::Value(doubleV));
  PatchDocumentOptions patchOptions(DocumentMask("total_consumption"), DocumentMask(), Precondition());
  
  String payload = docs.patch(userClient, Firestore::Parent(FIREBASE_PROJECT_ID), "devices/" + DEVICE_NAME, patchOptions, doc);

  if (userClient.lastError().code() == 0){

    Serial.print("Total Consumption updated to ");
    Serial.println(totalVolume);
  }
  else
    Firebase.printf("Error Updating total_consumption, msg: %s, code: %d\n", userClient.lastError().message().c_str(), userClient.lastError().code());
}

// Check for how long the water has been flowing continously. 
// If it exceeds the defined maximum duration, then there is a water leakage so close the valve,.
// update the valve status and send alert.
void checkWaterLeak(){

  if (flowInd){

    flowInd = false;

    if (currTime - startFlowTime > maxFlow){

      valveStatus = false;
      
      //Update the valve status in LCD
      lcd.setCursor(0,1);
      digitalWrite(VALVE_CONTROLLER_PIN, HIGH);
      lcd.print("Valve: CLOSED");
      Serial.println("valveStatus is FALSE");

      if (appUser.ready()) { //Apply update whe firebase app is ready

        updateValveStatus();
        sendLeakLog((ntpTime + currTime/1000), (ntpTime + startFlowTime/1000));
      }
    }
  }
  else{

    startFlowTime = currTime;

  }

}

//Update the database with the new valve status
void updateValveStatus(){

  Values::BooleanValue boolV(valveStatus);
  Document<Values::Value> doc("valve_status", Values::Value(boolV));
  PatchDocumentOptions patchOptions(DocumentMask("valve_status"), DocumentMask(), Precondition());

  String payload = docs.patch(userClient, Firestore::Parent(FIREBASE_PROJECT_ID), "devices/" + DEVICE_NAME, patchOptions, doc);

  if (userClient.lastError().code() == 0){

    Serial.print("Valve_status updated to ");

    if (valveStatus == true)
      Serial.println("TRUE");
    else
      Serial.println("FALSE");
  }
  else
    Firebase.printf("Error updating valve status, msg: %s, code: %d\n", userClient.lastError().message().c_str(), userClient.lastError().code());
}

//Send water leak log details
void sendLeakLog(unsigned long currentTime, unsigned long startTime){

  String timeStamp = getTimestampString(currentTime, 0);
  String startFlowTime = getTimestampString(startTime, 0);

  Values::StringValue timeStampStrV(timeStamp);
  Document<Values::Value> doc("time_stamp", Values::Value(timeStampStrV));

  Values::StringValue startFlowStrV(startFlowTime);
  doc.add("start_flow_time", Values::Value(startFlowStrV));

  Values::StringValue endFlowStrV(timeStamp);
  doc.add("end_flow_time", Values::Value(endFlowStrV));

  // Sync call which waits until the payload was received.
  String payload = docs.createDocument(userClient, Firestore::Parent(FIREBASE_PROJECT_ID), "devices/" + DEVICE_NAME + "/water_leak_log", DocumentMask(), doc);

  if (userClient.lastError().code() == 0)
    return;
  else
    Firebase.printf("Error, msg: %s, code: %d\n", userClient.lastError().message().c_str(), userClient.lastError().code());

}

// Function to get NTP server time.
uint32_t get_ntp_time(){
  uint32_t ts = 0;
  Serial.print("Getting time from NTP server... ");
  int max_try = 10, retry = 0;
  while (time(nullptr) < FIREBASE_DEFAULT_TS && retry < max_try)
  {
      configTime(3 * 3600, 0, "pool.ntp.org");
      unsigned long m = millis();
      while (time(nullptr) < FIREBASE_DEFAULT_TS && millis() - m < 10 * 1000)
      {
          delay(100);
          ts = time(nullptr);
      }
      Serial.print(ts == 0 ? " failed, retry... " : "");
      retry++;
  }
  ts = time(nullptr);

  Serial.println(ts > 0 ? "success" : "failed");

  return ts;
}


String getTimestampString(uint64_t sec, uint32_t nano)
{
    if (sec > 0x3afff4417f)
        sec = 0x3afff4417f;

    if (nano > 0x3b9ac9ff)
        nano = 0x3b9ac9ff;

    time_t now;
    struct tm ts;
    char buf[80];
    now = sec;
    ts = *localtime(&now);

    String format = "%Y-%m-%dT%H:%M:%S";

    if (nano > 0)
    {
        String fraction = String(double(nano) / 1000000000.0f, 9);
        fraction.remove(0, 1);
        format += fraction;
    }
    format += "Z";

    strftime(buf, sizeof(buf), format.c_str(), &ts);
    return buf;
}