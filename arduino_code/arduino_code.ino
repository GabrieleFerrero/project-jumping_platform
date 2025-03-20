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
float load_cells_tare[] = {0.0,0.0}; // 0:LX, 1:RX
float load_cells_calibration_factor[] = {1.0,1.0}; // 0:LX, 1:RX
// --------------------- //

// PROTOTYPES FUNCTIONAL //
float get_load_cells_adc_value(int id_load_cell, int num_samples);
float get_load_cells_adc_average_reading(int num_samples);
// --------------------- //

void setup() {
  // --INITIALIZATION-- //
  load_cells_adc.InitializeADC();
  load_cells_adc.setPGA(PGA_64); // Maximum sensitivity for weak signals
  load_cells_adc.setDRATE(DRATE_30000SPS); // Maximum sampling rate (30k SPS)
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
            lc.add("LX");
            lc.add("RX");
            doc_out["code"] = "OK";
          }else if(doc_in["command"]=="get_data"){
            JsonObject response = doc_out.createNestedObject("response");
            JsonObject lc = response.createNestedObject("lc");

            float value_LX = 5;//get_load_cells_adc_value(0, 10);
            if (isnan(value_LX)){
              lc["LX"] = nullptr;
            }else{
              lc["LX"] = value_LX;
            }
            float value_RX = 5;//get_load_cells_adc_value(1, 10);
            if (isnan(value_RX)){
              lc["RX"] = nullptr;
            }else{
              lc["RX"] = value_RX;
            }
            doc_out["code"] = "OK";
          }else if(doc_in["command"]=="scale_tare"){
            load_cells_adc.sendDirectCommand(SELFCAL);

            JsonObject response = doc_out.createNestedObject("response");
            JsonObject lc = response.createNestedObject("lc");

            float value_LX = 5;//get_load_cells_adc_value(0, 200);
            if (isnan(value_LX)){
              lc["LX"] = "ERROR";
            }else{
              load_cells_tare[0]=value_LX;
              lc["LX"] = "OK";
            }
            float value_RX = 5;//get_load_cells_adc_value(1, 200);
            if (isnan(value_RX)){
              lc["RX"] = "ERROR";
            }else{
              load_cells_tare[1]=value_RX;
              lc["RX"] = "OK";
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

float get_load_cells_adc_value(int id_load_cell, int num_samples){

  switch(id_load_cell){
    case 0:
      load_cells_adc.setMUX(DIFF_0_1); // Select the first differential channel
      break;
    case 1:
      load_cells_adc.setMUX(DIFF_2_3); // Select the first differential channel
      break;
    default:
      return NAN;
      break;
  }

  float raw_data = get_load_cells_adc_average_reading(num_samples);
  if (isnan(raw_data)){
    return NAN;
  }else{
    return (raw_data*load_cells_calibration_factor[id_load_cell])-load_cells_tare[id_load_cell];
  }

}

float get_load_cells_adc_average_reading(int num_samples){
  double sum = 0.0;
  for (int i=0; i<num_samples; i++) {
    long raw_data = load_cells_adc.readSingleContinuous();
    float voltage_value = load_cells_adc.convertToVoltage(raw_data);
    if(isnan(voltage_value)){
      return NAN;
    }else{
      sum += ((double)voltage_value);
    }
  }
  return ((float)(sum/((double)num_samples)));
}
