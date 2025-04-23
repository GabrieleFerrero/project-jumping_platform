// -------LIBRARY------- //
#include <ArduinoJson.h>
#include <ADS1256.h>
// --------------------- //

// -------COSTANT------- //
#define DRDY_PIN 16
#define RESET_PIN 0
#define SYNC_PIN 0
#define CS_PIN 5
const float VREF = 2.5;
const int numberOfSamplesForScaleTare = 200;
const char* NAME_LC_LABELS[] = {"LC1", "LC2", "LC3", "LC4"};
// --------------------- //

// --GLOBAL VARIABLES--- //
StaticJsonDocument<350> docIn;
DeserializationError error;
enum LCLabels {LC1, LC2, LC3, LC4, NUMBER_LC_LABELS};
String serialInput = "";
bool readingStatus = false;
bool sendJson = true;
ADS1256 loadCellsADC(DRDY_PIN, RESET_PIN, SYNC_PIN, CS_PIN, VREF); // DRDY, RESET, SYNC(PDWN), CS, VREF
double loadCellsTare[] = {0.0,0.0,0.0,0.0};
double loadCellsCalibrationFactor[] = {0.0046771144,0.0046056742,0.0046736458,0.0046137667};
int singleEndedChannels[8] = {SING_0, SING_1, SING_2, SING_3, SING_4, SING_5, SING_6, SING_7}; //Array to store the single-ended channels
int differentialChannels[4] = {DIFF_0_1, DIFF_2_3, DIFF_4_5, DIFF_6_7}; //Array to store the differential channels
int pgaValues[7] = {PGA_1, PGA_2, PGA_4, PGA_8, PGA_16, PGA_32, PGA_64};
int drateValues[16] ={
  DRATE_30000SPS,
  DRATE_15000SPS,
  DRATE_7500SPS,
  DRATE_3750SPS,
  DRATE_2000SPS,
  DRATE_1000SPS,
  DRATE_500SPS,
  DRATE_100SPS,
  DRATE_60SPS,
  DRATE_50SPS,
  DRATE_30SPS,
  DRATE_25SPS,
  DRATE_15SPS,
  DRATE_10SPS,
  DRATE_5SPS,
  DRATE_2SPS
};
// --------------------- //

// PROTOTYPES FUNCTIONAL //

// --------------------- //

void setup() {
  // --INITIALIZATION-- //
  loadCellsADC.InitializeADC();
  loadCellsADC.setPGA(PGA_64); // Maximum sensitivity for weak signals
  loadCellsADC.setDRATE(DRATE_15000SPS); // Maximum sampling rate (30k SPS)
  loadCellsADC.sendDirectCommand(SELFCAL);
  delay(100);
  // --------------------- //
  
  Serial.begin(921600); // 460800
  while (!Serial) {
    ; // wait for serial port to connect. Needed for native USB
  } 
}

void loop() {
  if(Serial.available() != 0){
    char input = Serial.read();

    switch(input){
      case '?': // reset
      {
        serialInput = "";
        break;
      }
      case '\r':
      {
        break;
      }
      case '\n':
      {
        break;
      }
      case '!':
      {
        sendJson = true;

        StaticJsonDocument<500> docOut;

        error = deserializeJson(docIn, serialInput);
        if (error) {
          docOut["code"] = "ERROR"; 
          docOut["serialInput"] = serialInput;
        }else{
          //Serial.println(serial_input);  

          if (!docIn.containsKey("command") || docIn["command"].isNull()) {
            docOut["code"] = "ERROR";
            docOut["message"] = serialInput;
          }else{
          
            docOut["command"] = docIn["command"];

            if(docIn["command"]=="set_state_digital_pin"){
              int pin = docIn["pin"];
              int value = docIn["value"];
              digitalWrite(pin, value);
            }else if(docIn["command"]=="set_state_analog_pin"){
              int pin = docIn["pin"];
              int value = docIn["value"];
              analogWrite(pin, value);
            }else if(docIn["command"]=="get_state_digital_pin"){
              int pin = docIn["pin"];
              int value = digitalRead(pin);
              docOut["value"] = value;
            }else if(docIn["command"]=="get_state_analog_pin"){
              int pin = docIn["pin"];
              int value = analogRead(pin);
              docOut["value"] = value;
            }else if(docIn["command"]=="get_info"){
              JsonArray lc = docOut.createNestedArray("lc");

              for (int i = 0; i < NUMBER_LC_LABELS; i++) {
                lc.add(NAME_LC_LABELS[i]);
              }

            }else if(docIn["command"]=="start_reading"){
              readingStatus=true;
              sendJson = false;

            }else if(docIn["command"]=="scale_tare"){
              JsonObject lc = docOut.createNestedObject("lc");

              loadCellsADC.stopConversion();

              loadCellsADC.sendDirectCommand(SELFCAL);
              delay(100);

              double loadCellsRawData[] = {0.0,0.0,0.0,0.0};

              for(int i=0; i<numberOfSamplesForScaleTare; i++){
                for (int j=0; j<NUMBER_LC_LABELS; j++) {
                  loadCellsRawData[j] += loadCellsADC.cycleDifferential()/numberOfSamplesForScaleTare;
                }
              }

              for(int i=0; i<NUMBER_LC_LABELS; i++){
                if (isnan(loadCellsRawData[i])){
                  lc[NAME_LC_LABELS[i]] = "ERROR";
                }else{
                  loadCellsTare[i] = loadCellsRawData[i];
                  lc[NAME_LC_LABELS[i]] = "OK";
                }              
              }

              loadCellsADC.stopConversion();
              
              
            }else if(docIn["command"]=="is_alive"){

            }else if(docIn["command"]=="stop_reading"){
              loadCellsADC.stopConversion();
              readingStatus=false;
            }else{
              docOut["code"] = "ERROR"; 
            }

          }
        }

        serialInput = "";

        if(sendJson){
          Serial.print("?");
          serializeJson(docOut, Serial);
          Serial.println("!");
        }


        break;
      }
      default:
      {
        serialInput += input;
        break;
      }
    }
  }

  // ---------------------- FUNCTIONS TO RECALL -------------------------

  if(readingStatus){

    Serial.print("{\"values\":{");
    for(int i = 0; i < NUMBER_LC_LABELS; i++){
      Serial.print("\"");
      Serial.print(NAME_LC_LABELS[i]);
      Serial.print("\":");

      double value = (((double)loadCellsADC.cycleDifferential())-loadCellsTare[i])*loadCellsCalibrationFactor[i];
      
      if(isnan(value)){
        Serial.print("null");
      }else{
        Serial.print(value, 3);
      }
     
      if (i < NUMBER_LC_LABELS-1) Serial.print(",");
    }
    Serial.print("},\"time\":");
    Serial.print(((double)micros())/1000000.0, 6);
    Serial.print("}!");
    
  }



}