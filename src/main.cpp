#include <Arduino.h>

#define USE_N2K_CAN 11
#include <NMEA2000_CAN.h>
#include <N2kMessages.h>

#include <Wire.h>
#include <Adafruit_INA219.h>

#define LED_PIN PC13
#define SERIAL_DEBUG
const char software_version[] = "0.1.0.1";
const char serial_number[] = "8cc3145208774f6897e52d7b3a36af43";

// INA219 Address Matrix:
// A0 | A1 | Address |
// 0  | 0  | 0x40    |
// 1  | 0  | 0x41    |
// 0  | 1  | 0x44    |
// 1  | 1  | 0x45    |
Adafruit_INA219 fuel_INA219(0x40);  // A0=0, A1=0
Adafruit_INA219 trim_INA219(0x41);  // A0=1, A1=0
Adafruit_INA219 aux_INA219(0x44);   // A0=0, A1=1
Adafruit_INA219 aux2_INA219(0x45);  // A0=1, A1=1

tN2kSyncScheduler FuelLevelScheduler(false, 2500, 500);
tN2kSyncScheduler TrimPercentScheduler(false, 2500, 505);
tN2kSyncScheduler Aux1Scheduler(false, 2500, 510);
tN2kSyncScheduler Aux2Scheduler(false, 2500, 515);

double fuel_capacity = 246;
float fuel_min_resistance = 240.0;
float fuel_max_resistance = 33.0;
float fuel_slope = 100 / (fuel_max_resistance - fuel_min_resistance);

float trim_min_resistance = 10.0;
float trim_max_resistance = 253.6;
float trim_slope = 100 / (trim_max_resistance - trim_min_resistance);

Stream *read_stream = &Serial1;
Stream *forward_stream = &Serial1;

void onN2kOpen(){
  FuelLevelScheduler.UpdateNextTime();
  TrimPercentScheduler.UpdateNextTime();
  Aux1Scheduler.UpdateNextTime();
  Aux2Scheduler.UpdateNextTime();
  Serial.println("NMEA2000 Opened");
}

double ReadFuelLevel() {
  double bus_voltage = fuel_INA219.getBusVoltage_V();
  double shunt_mV = fuel_INA219.getShuntVoltage_mV();
  double load_voltage = bus_voltage + (shunt_mV / 1000);
  double current_mA = fuel_INA219.getCurrent_mA() * (49.47 / 62.900);
  double sender_resistance = bus_voltage / (current_mA / 1000);
  double fuel_level_percent = fuel_slope * (sender_resistance - fuel_min_resistance);
  #ifdef SERIAL_DEBUG
  Serial.println("Fuel Data:");
  Serial.println("\tVoltage:\t| " + String(bus_voltage) + "V;\r\n\tShunt Voltage:\t| " + shunt_mV + "mV;\r\n\tSource Voltage:\t| " + load_voltage + "V;\r\n\tCurrent:\t| " + current_mA + "mA;\r\n\tResistance:\t| " + sender_resistance + "ohm;\r\n\tFuel Level:\t| " + fuel_level_percent + "\%.\n");
  #endif
  return fuel_level_percent;
}

double ReadTrimPercent() {
  double bus_voltage = trim_INA219.getBusVoltage_V();
  double shunt_mV = trim_INA219.getShuntVoltage_mV();
  double load_voltage = bus_voltage + (shunt_mV / 1000);
  double current_mA = trim_INA219.getCurrent_mA() * (49.08 / 61.200);
  double sender_resistance = bus_voltage / (current_mA / 1000);
  double trim_percent = trim_slope * (sender_resistance - trim_min_resistance);
  #ifdef SERIAL_DEBUG
  Serial.println("Trim Data:");
  Serial.println("\tVoltage:\t| " + String(bus_voltage) + "V;\r\n\tShunt Voltage:\t| " + shunt_mV + "mV;\r\n\tSource Voltage:\t| " + load_voltage + "V;\r\n\tCurrent:\t| " + current_mA + "mA;\r\n\tResistance:\t| " + sender_resistance + "ohm;\r\n\tTrim Level:\t| " + trim_percent + "\%\r\n");
  #endif
  return trim_percent;
}

double ReadAux1() {
  double bus_voltage = aux_INA219.getBusVoltage_V();
  double shunt_mV = aux_INA219.getShuntVoltage_mV();
  double load_voltage = bus_voltage + (shunt_mV / 1000);
  double current_mA = aux_INA219.getCurrent_mA() * (49.08 / 61.200);
  double sender_resistance = bus_voltage / (current_mA / 1000);
  // double trim_percent = trim_slope * (sender_resistance - trim_min_resistance);
  #ifdef SERIAL_DEBUG
  Serial.println("Aux1 Data:");
  Serial.println("\tVoltage:\t| " + String(bus_voltage) + "V;\r\n\tShunt Voltage:\t| " + shunt_mV + "mV;\r\n\tSource Voltage:\t| " + load_voltage + "V;\r\n\tCurrent:\t| " + current_mA + "mA;\r\n\tResistance:\t| " + sender_resistance + "ohm;\r\n");
  #endif
  // return trim_percent;
  return sender_resistance;
}

double ReadAux2() {
  double bus_voltage = aux2_INA219.getBusVoltage_V();
  double shunt_mV = aux2_INA219.getShuntVoltage_mV();
  double load_voltage = bus_voltage + (shunt_mV / 1000);
  double current_mA = aux2_INA219.getCurrent_mA() * (49.08 / 61.200);
  double sender_resistance = bus_voltage / (current_mA / 1000);
  // double trim_percent = trim_slope * (sender_resistance - trim_min_resistance);
  #ifdef SERIAL_DEBUG
  Serial.println("Aux2 Data:");
  Serial.println("\tVoltage:\t| " + String(bus_voltage) + "V;\r\n\tShunt Voltage:\t| " + shunt_mV + "mV;\r\n\tSource Voltage:\t| " + load_voltage + "V;\r\n\tCurrent:\t| " + current_mA + "mA;\r\n\tResistance:\t| " + sender_resistance + "ohm;\r\n");
  #endif
  // return trim_percent;
  return sender_resistance;
}

void SendN2kMsg (tN2kMsg message) {
  NMEA2000.SendMsg(message);
  digitalWrite(LED_PIN, !digitalRead(LED_PIN));
}

void SendN2kFuelLevel() {
  tN2kMsg N2kMsg;
  
  if ( FuelLevelScheduler.IsTime() ) {
    FuelLevelScheduler.UpdateNextTime();
    SetN2kFluidLevel(
      N2kMsg,
      0,
      N2kft_Fuel,
      ReadFuelLevel(),
      fuel_capacity
    );
    SendN2kMsg(N2kMsg);
  }
}

void SendN2kTrimPercent() {
  tN2kMsg N2kMsg;
  if ( TrimPercentScheduler.IsTime() ) {
    TrimPercentScheduler.UpdateNextTime();
    SetN2kEngineParamRapid(
      N2kMsg,
      0,
      N2kDoubleNA,
      N2kDoubleNA,
      ReadTrimPercent()
    );
    SendN2kMsg(N2kMsg);
  }
}

// Placeholder for future INA219 sensor readings.  All NMEA2k functions disabled.
void SendN2kAux1() {
  // tN2kMsg N2kMsg;
  
  if ( Aux1Scheduler.IsTime() ) {
    Aux1Scheduler.UpdateNextTime();
    ReadAux1();
    // SetN2kFluidLevel(
    //   N2kMsg,
    //   0,
    //   N2kft_Fuel,
    //   ReadFuelLevel(),
    //   fuel_capacity
    // );
    // SendN2kMsg(N2kMsg);
  }
}

// Placeholder for future INA219 sensor readings.  All NMEA2k functions disabled.
void SendN2kAux2() {
  // tN2kMsg N2kMsg;
  
  if ( Aux2Scheduler.IsTime() ) {
    Aux2Scheduler.UpdateNextTime();
    ReadAux2();
    // SetN2kFluidLevel(
    //   N2kMsg,
    //   0,
    //   N2kft_Fuel,
    //   ReadFuelLevel(),
    //   fuel_capacity
    // );
    // SendN2kMsg(N2kMsg);
  }
}

void HandleStreamN2kMsg(const tN2kMsg &message) {
  message.Print(&Serial);
}

void setup() {
  Serial.begin(115200);
  while (!Serial);
  Serial.println("Starting Gauge Sender V" + String(software_version));
  Serial.println("Serial Number: " + String(serial_number));
  pinMode(LED_PIN, OUTPUT);
  fuel_INA219.begin();
  trim_INA219.begin();
  aux_INA219.begin();
  aux2_INA219.begin();

  NMEA2000.SetN2kCANSendFrameBufSize(250);
  NMEA2000.SetN2kCANReceiveFrameBufSize(250);
  NMEA2000.SetProductInformation(
    serial_number,   // Serial Number
    100,                                  // Manufacturer's product code
    "Scott's Gauge Sender",               // Manufacturer's Model ID (max 33 chars)
    software_version,                     // Manufacturer's Software version code (max 40 chars)
    "0.1.0.0"                             // Manufacturer's Model version (max 24 chars)
  );
  NMEA2000.SetDeviceInformation(
      3948502847,     // Unique number. Use e.g. Serial number.
      170,        // Device function=Analog to NMEA 2000 Gateway. See codes on http://www.nmea.org/Assets/20120726%20nmea%202000%20class%20&%20function%20codes%20v%202.00.pdf
      35,         // Device class=Inter/Intranetwork Device. See codes on http://www.nmea.org/Assets/20120726%20nmea%202000%20class%20&%20function%20codes%20v%202.00.pdf
      122         // Just choose a free from code list on http://www.nmea.org/Assets/20121020%20nmea%202000%20registration%20list.pdf
  );
  NMEA2000.SetForwardStream(forward_stream);
  NMEA2000.SetMode(tNMEA2000::N2km_ListenAndNode);
  #ifdef SERIAL_DEBUG
  NMEA2000.SetForwardType(tNMEA2000::fwdt_Text);
  #endif
  NMEA2000.SetForwardOwnMessages(true);
  NMEA2000.SetMsgHandler(HandleStreamN2kMsg);
  NMEA2000.SetOnOpen(onN2kOpen);
  NMEA2000.Open();
}

unsigned long parse_time = millis();

void loop() {
  if ( millis() - parse_time > 1 ) {
    parse_time = millis();
    NMEA2000.ParseMessages();
  }
  SendN2kFuelLevel();
  SendN2kTrimPercent();
  SendN2kAux1();
  SendN2kAux2();
}