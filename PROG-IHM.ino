

/**************************************                                                                              **************************************/
/************************************** Programming for Modbus connection with Delta HMI, local dashboard (OFFLINE)  **************************************/
/**************************************                                                                              **************************************/


#include <ArduinoModbus.h>                            // OPTA Modbus Library
#include <ArduinoRS485.h>                             // OPTA Modbus Library
unsigned int ctr = 0 ;                                // Variable named CTR starting at 0
float v = 0;                                          // Variable named V starting at 0
float p = 0;                                          // Variable named P starting at 0
float p1 = 0;                                         // Variable named P1 starting at 0
float voltageA0 = 0;                                  // Variable named VOLTAGEA0 starting at 0
float voltageA1 = 0;                                  // Variable named VOLTAGEA1 starting at 0
int liga = 0;                                         // Variable named LIGA starting at 0
int b1 = 0;                                           // Variable named B1 starting at 0
int b2 = 0;                                           // Variable named B2 starting at 0
constexpr auto baudrate{ 19200 };                     // Modbus Baud Rate configured to be 19200
constexpr auto btime{ 1.0f / baudrate };              // Modbus Configuration
constexpr auto predl{ btime * 9.6f * 3.5f * 1e6 };    // Modbus Configuration
constexpr auto postdl{ btime * 9.6f * 3.5f * 1e6 };   // Modbus Configuration

void setup() {
  Serial.begin(115200);                                   // Serial Port configured at 115200
  delay(400);                                             // Wait 400ms
  RS485.setDelays(predl, postdl);                         // Modbus Configuration        
  if (!ModbusRTUClient.begin(baudrate, SERIAL_8N2)) {     // Modbus Configuration ("NONE" parity and Stop bits 2)
    Serial.println("Erro Modbus");                        // Erro Modbus
    while (1)                                             // Loop to start Modbus reading
      ;
  }

  analogReadResolution(12);                               // Analog readout configured for 12 bits
  pinMode(LED_D0, OUTPUT);                                // LED_D0 configured to be output
  pinMode(D0, OUTPUT);                                    // D0 configured to be output
  pinMode(LED_USER, OUTPUT);                              // LED_USER configured to be output
  pinMode(LEDR, OUTPUT);                                  // LEDR configured to be output
}

void loop() {
  ctr++;                                                                // starts counting in the CTR variable (this count is not displayed in the HMI)
  ModbusRTUClient.beginTransmission(2, HOLDING_REGISTERS, 1, 10);       // Modbus Configuration
  ModbusRTUClient.write(ctr);                                           // Prints value of the CTR variable on the HMI (not shown on the display)
  ModbusRTUClient.write(2);                                             // Prints information in a position 2 variable (used only for tests)
  ModbusRTUClient.write(3);                                             // Prints information in a position 3 variable (used only for tests)
  ModbusRTUClient.write(v);                                             // Prints value of the V variable on the HMI
  ModbusRTUClient.write(p1);                                            // Prints value of the P1 variable on the HMI
  ModbusRTUClient.write(voltageA0);                                     // Prints value of the VOLTAGEA0 variable on the HMI
  ModbusRTUClient.write(liga);                                          // Prints value of the LIGA variable on the HMI
  ModbusRTUClient.write(b1);                                            // Prints value of the B1 variable on the HMI
  ModbusRTUClient.write(b2);                                            // Prints value of the B2 variable on the HMI
  ModbusRTUClient.write(voltageA1);                                     // Prints value of the VOLTAGEA1 variable on the HMI
  auto estadoSaida = ModbusRTUClient.coilRead(2,1);                     // Reading the button on the HMI to give a command to the OPTA
  

  Serial.println("Conectado com sucesso");                              // Print result on connection

  if (!ModbusRTUClient.endTransmission()) {
    Serial.print("failed! ");
    Serial.println(ModbusRTUClient.lastError());
  }

  //0x09BA - Active Power Total 32490-32491, IEEE 754 T_Float, x 1W
  p = readdata(0x21, 0X9BA);                                                          // Modbus ID and address configuration for reading
  //0x09C4 - Tensione 32500-32501, IEEE 754 T_Float, x 
  v = readdata(0x21, 0X9C4);                                                          // Modbus ID and address configuration for reading
  // 0x9D4 - corrente 32516-32517, IEEE 754 T_Float, x 1A
  float i = readdata(0x21, 0X9D4);                                                    // Modbus ID and address configuration for reading, but not being used and shown on the HMI display

  Serial.println(String(p, 1) + "W " + String(v, 1) + "V " + String(i, 3) + "A ");    // Prints the values ​​read from the variables above in Serial
  delay(400);                                                                         // Wait 400ms

  p1 = p*10;                                                                          // Multiplies the value of variable P by 10 and stores this value in variable P1

  int sensorValueA0 = analogRead(A0);                                                 // Analog reading of OPTA port A0.
  voltageA0 = sensorValueA0 * (3.0 / 4095.0)/ 0.3;                                    // Stores the values ​​of the sensorValueA0 variable in the VOLTAGEA0 variable

  int sensorValueA1 = analogRead(A1);                                                 // Analog reading of OPTA port A0.
  voltageA1 = sensorValueA1 * (3.0 / 4095.0)/ 0.3;                                    //Stores the values ​​of the sensorValueA1 variable in the VOLTAGEA1 variable

  Serial.print("I1 value: ");                                                         // Prints the results in the Serial corresponding to the read variable
   Serial.print(sensorValueA0);                                                       // Prints the results in the Serial corresponding to the read variable
   Serial.print(" corresponding to ");                                                // Prints the results in the Serial corresponding to the read variable
   Serial.print(voltageA0, 5);                                                        // Prints the results in the Serial corresponding to the variable read with 5 decimal places
   Serial.println("Volts");                                                           // Prints the results in the Serial corresponding to the read variable

  Serial.print("I2 value: ");                                                         // Prints the results in the Serial corresponding to the read variable                 
   Serial.print(sensorValueA1);                                                       // Prints the results in the Serial corresponding to the read variable
   Serial.print(" corresponding to ");                                                // Prints the results in the Serial corresponding to the read variable
   Serial.print(voltageA1, 5);                                                        // Prints the results in the Serial corresponding to the variable read with 5 decimal places
   Serial.println("Volts");                                                           // Prints the results in the Serial corresponding to the read variable


   if(estadoSaida){                                                                   // If the outputstate variable changes its initial value:
    digitalWrite(LED_USER, HIGH);                                                     // LED_USER ON
    digitalWrite(LEDR, LOW);                                                          // LEDR OFF
    delay(300);                                                                       // Wait 300ms
    digitalWrite(LED_USER, LOW);                                                      // LED_USER OFF
    digitalWrite(LEDR, HIGH);                                                         // LEDR ON
    delay(50);                                                                        // Wait 50ms
    digitalWrite(LED_D0, LOW);                                                        // LED_D0 OFF
    digitalWrite(D0, LOW);                                                            // D0 OFF
   }else{                                                                             // IF NOT
    digitalWrite(LED_D0, HIGH);                                                       // LED_D0 ONN
    digitalWrite(D0, HIGH);                                                           // D0 HIGH
    digitalWrite(LED_USER, HIGH);                                                     // LED_USER ON
    digitalWrite(LEDR, HIGH);                                                         // LEDR ON
   }


}


float readdata(int addr, int reg) {                                            // Modbus Configuration
  float res = 0.0;                                                             // Modbus Configuration
  if (!ModbusRTUClient.requestFrom(addr, INPUT_REGISTERS, reg, 2)) {           // Modbus Configuration
    Serial.println("Erro de comunicação");                                     // Modbus Configuration
    Serial.println(ModbusRTUClient.lastError());                               // Modbus Configuration
  } else {
    uint16_t word1 = ModbusRTUClient.read();                                   // Modbus Configuration
    uint16_t word2 = ModbusRTUClient.read();                                   // Modbus Configuration
    uint32_t parz = word1 << 16 | word2;                                       // Modbus Configuration
    res = *(float *)&parz;                                                     // Modbus Configuration
  }
  return res;
}
