// -------LIBRARY------- //
#include <ArduinoJson.h>
#include <ADS1256.h>
// --------------------- //

// -------COSTANT------- //
#define DRDY_PIN 16
#define RESET_PIN 17
#define SYNC_PIN 0
#define CS_PIN 5
const float VREF = 2.5;
// --------------------- //

// --GLOBAL VARIABLES--- //
String serial_input = "";
ADS1256 load_cells_adc(DRDY_PIN, RESET_PIN, SYNC_PIN, CS_PIN, VREF); // DRDY, RESET, SYNC(PDWN), CS, VREF
double load_cells_tare[] = {0.0};
double load_cells_calibration_factor[] = {4.5227};
int singleEndedChannels[8] = {SING_0, SING_1, SING_2, SING_3, SING_4, SING_5, SING_6, SING_7}; //Array to store the single-ended channels
int differentialChannels[4] = {DIFF_0_1, DIFF_2_3, DIFF_4_5, DIFF_6_7}; //Array to store the differential channels
int pgaValues[7] = {PGA_1, PGA_2, PGA_4, PGA_8, PGA_16, PGA_32, PGA_64};
int drateValues[16] =
{
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

enum LCLabels {
    LX,
    NUMBER_LC_LABELS
};

const char* NameLCLabels[] = {
    "LX"
};
// --------------------- //

// PROTOTYPES FUNCTIONAL //
double get_load_cells_adc_value(int id_load_cell, int num_samples);
void set_mux_channel(char input_mode, int input_channel);
double get_load_cells_adc_average_reading(int num_samples);
// --------------------- //

void setup() {
  // --INITIALIZATION-- //
  load_cells_adc.InitializeADC();
  load_cells_adc.setPGA(PGA_64); // Maximum sensitivity for weak signals
  set_mux_channel('d', 0);
  load_cells_adc.setDRATE(DRATE_7500SPS); // Maximum sampling rate (30k SPS)
  load_cells_adc.sendDirectCommand(SELFCAL);
  delay(100);
  // --------------------- //

  Serial.begin(460800);
  while (!Serial) {
    ; // wait for serial port to connect. Needed for native USB
  } 
}

void loop() {
  StaticJsonDocument<350> doc_in;
  StaticJsonDocument<400> doc_out;
  DeserializationError error;

  
  if(Serial.available() != 0){
    char input = Serial.read();

    switch(input){
      case '?': // reset
        serial_input = "";
        break;
      case '\r':
        break;
      case '\n':
        break;
      case '!':

        //Serial.println(serial_input);  

        error = deserializeJson(doc_in, serial_input);
        if (error) {
          doc_out["code"] = "ERROR"; 
        }else{
          //Serial.println(serial_input);          

          if(doc_in["command"]=="set_state_digital_pin"){
            int pin = doc_in["pin"];
            int value = doc_in["value"];
            digitalWrite(pin, value);
            doc_out["code"] = "OK";
          }else if(doc_in["command"]=="set_state_analog_pin"){
            int pin = doc_in["pin"];
            int value = doc_in["value"];
            analogWrite(pin, value);
            doc_out["code"] = "OK";
          }else if(doc_in["command"]=="get_state_digital_pin"){
            int pin = doc_in["pin"];
            int value = digitalRead(pin);
            doc_out["value"] = value;
            doc_out["code"] = "OK";
          }else if(doc_in["command"]=="get_state_analog_pin"){
            int pin = doc_in["pin"];
            int value = analogRead(pin);
            doc_out["value"] = value;
            doc_out["code"] = "OK";
          }else if(doc_in["command"]=="get_info"){
            JsonObject response = doc_out.createNestedObject("response");
            JsonArray lc = response.createNestedArray("lc");

            for (int i = 0; i < NUMBER_LC_LABELS; i++) {
              lc.add(NameLCLabels[i]);
            }

            doc_out["code"] = "OK";
          }else if(doc_in["command"]=="get_data"){
            JsonObject response = doc_out.createNestedObject("response");
            JsonObject lc = response.createNestedObject("lc");

            for (int i = 0; i < NUMBER_LC_LABELS; i++) {
              double value = get_load_cells_adc_value(i, 30);
              if (isnan(value)){
                lc[NameLCLabels[i]] = nullptr;
              }else{
                lc[NameLCLabels[i]] = value;
              }
            }
         
            doc_out["code"] = "OK";
          }else if(doc_in["command"]=="scale_tare"){
            JsonObject response = doc_out.createNestedObject("response");
            JsonObject lc = response.createNestedObject("lc");

            load_cells_adc.sendDirectCommand(SELFCAL);
            delay(100);

            for (int i = 0; i < NUMBER_LC_LABELS; i++) {
              double value = get_load_cells_adc_average_reading(200);
              if (isnan(value)){
                lc[NameLCLabels[i]] = "ERROR";
              }else{
                load_cells_tare[i]=value;
                lc[NameLCLabels[i]] = "OK";
              }
            }

            doc_out["code"] = "OK";
          }else if(doc_in["command"]=="is_alive"){
            doc_out["code"] = "OK";
          }else{
            doc_out["code"] = "ERROR"; 
          }
        }

        serial_input = "";

        Serial.print("?");
        serializeJson(doc_out, Serial);
        Serial.println("!");

        break;
      default:
        serial_input += input;
        break;
    }
  }


}

double get_load_cells_adc_value(int id_load_cell, int num_samples){

  //set_mux_channel('d', 0);

  double raw_data = get_load_cells_adc_average_reading(num_samples);
  if (isnan(raw_data)){
    return NAN;
  }else{
    return (raw_data-load_cells_tare[id_load_cell])*load_cells_calibration_factor[id_load_cell];
  }

}

void set_mux_channel(char input_mode, int input_channel){
  switch(input_mode){
    case 's':
      load_cells_adc.setMUX(singleEndedChannels[input_channel]);
      break;
    case 'd':
      load_cells_adc.setMUX(differentialChannels[input_channel]);
      break;
    default:
      break;
  }
  delay(50);
}

double get_load_cells_adc_average_reading(int num_samples){
  double sum = 0.0;
  for (int i=0; i<num_samples; i++) {
    double raw_data = load_cells_adc.readSingle();//readSingleContinuous();
    //Serial.println(raw_data, 10);
    // delay(10);
    if(isnan(raw_data)){
      return NAN;
    }else{
        sum += raw_data/1000; // 1000 is scale factor
      }
  }
  //load_cells_adc.stopConversion();
  double avg = (sum/((double)num_samples));
  return ((double)round(avg));
}
