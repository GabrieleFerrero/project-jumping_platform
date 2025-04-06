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
const char* NAME_LC_LABELS[] = {"LX"};
// --------------------- //

// --GLOBAL VARIABLES--- //
StaticJsonDocument<350> docIn;
DeserializationError error;
enum LCLabels {LX, NUMBER_LC_LABELS};
String serialInput = "";
ADS1256 loadCellsADC(DRDY_PIN, RESET_PIN, SYNC_PIN, CS_PIN, VREF); // DRDY, RESET, SYNC(PDWN), CS, VREF
double loadCellsRawData[] = {0.0};
double loadCellsTare[] = {0.0};
double loadCellsCalibrationFactor[] = {0.0045227};
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
void get_load_cells_force(int numSamples);
void set_mux_channel(char inputMode, int inputChannel);
void get_load_cells_adc_average_reading(int numSamples);
// --------------------- //

void setup() {
  // --INITIALIZATION-- //
  loadCellsADC.InitializeADC();
  loadCellsADC.setPGA(PGA_64); // Maximum sensitivity for weak signals
  loadCellsADC.setDRATE(DRATE_7500SPS); // Maximum sampling rate (30k SPS)
  loadCellsADC.sendDirectCommand(SELFCAL);
  delay(100);
  // --------------------- //

  Serial.begin(460800);
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
        //Serial.println(serial_input);  

        StaticJsonDocument<500> docOut;

        error = deserializeJson(docIn, serialInput);
        if (error) {
          docOut["code"] = "ERROR"; 
        }else{
          //Serial.println(serial_input);          

          if(docIn["command"]=="set_state_digital_pin"){
            int pin = docIn["pin"];
            int value = docIn["value"];
            digitalWrite(pin, value);
            docOut["code"] = "OK";
          }else if(docIn["command"]=="set_state_analog_pin"){
            int pin = docIn["pin"];
            int value = docIn["value"];
            analogWrite(pin, value);
            docOut["code"] = "OK";
          }else if(docIn["command"]=="get_state_digital_pin"){
            int pin = docIn["pin"];
            int value = digitalRead(pin);
            docOut["value"] = value;
            docOut["code"] = "OK";
          }else if(docIn["command"]=="get_state_analog_pin"){
            int pin = docIn["pin"];
            int value = analogRead(pin);
            docOut["value"] = value;
            docOut["code"] = "OK";
          }else if(docIn["command"]=="get_info"){
            JsonObject response = docOut.createNestedObject("response");
            JsonArray lc = response.createNestedArray("lc");

            for (int i = 0; i < NUMBER_LC_LABELS; i++) {
              lc.add(NAME_LC_LABELS[i]);
            }

            docOut["code"] = "OK";
          }else if(docIn["command"]=="get_data"){
            JsonObject response = docOut.createNestedObject("response");
            JsonObject lc = response.createNestedObject("lc");
            JsonObject values = lc.createNestedObject("values");

            get_load_cells_force(7);
            for (int i = 0; i < NUMBER_LC_LABELS; i++) {
              if (isnan(loadCellsRawData[i])){
                values[NAME_LC_LABELS[i]] = nullptr;
              }else{
                values[NAME_LC_LABELS[i]] = loadCellsRawData[i];
              }
            }
            lc["time"] = millis();
         
            docOut["code"] = "OK";
          }else if(docIn["command"]=="scale_tare"){
            JsonObject response = docOut.createNestedObject("response");
            JsonObject lc = response.createNestedObject("lc");

            loadCellsADC.sendDirectCommand(SELFCAL);
            delay(100);

            get_load_cells_adc_average_reading(200);
            for(int i=0; i<NUMBER_LC_LABELS; i++){
              if (isnan(loadCellsRawData[i])){
                lc[NAME_LC_LABELS[i]] = "ERROR";
              }else{
                loadCellsTare[i] = loadCellsRawData[i];
                lc[NAME_LC_LABELS[i]] = "OK";
              }              
            }

            docOut["code"] = "OK";
          }else if(docIn["command"]=="is_alive"){
            docOut["code"] = "OK";
          }else{
            docOut["code"] = "ERROR"; 
          }
        }

        serialInput = "";

        Serial.print("?");
        serializeJson(docOut, Serial);
        Serial.println("!");

        break;
      }
      default:
      {
        serialInput += input;
        break;
      }
    }
  }


}

void get_load_cells_force(int numSamples){
  get_load_cells_adc_average_reading(numSamples);
  for(int i=0; i<NUMBER_LC_LABELS; i++){
    loadCellsRawData[i]=(loadCellsRawData[i]-loadCellsTare[i])*loadCellsCalibrationFactor[i];
  }
}

void set_mux_channel(char inputMode, int inputChannel){
  switch(inputMode){
    case 's':
      loadCellsADC.setMUX(singleEndedChannels[inputChannel]);
      break;
    case 'd':
      loadCellsADC.setMUX(differentialChannels[inputChannel]);
      break;
    default:
      break;
  }
  delay(50);
}

void get_load_cells_adc_average_reading(int numSamples){
  for(int i=0; i<NUMBER_LC_LABELS; i++){loadCellsRawData[i]=0.0;}
  for(int i=0; i<numSamples; i++){
    for (int j=0; j<NUMBER_LC_LABELS; j++) {
      loadCellsRawData[j] += loadCellsADC.cycleDifferential()/numSamples;
    }
  }
  loadCellsADC.stopConversion();
}
