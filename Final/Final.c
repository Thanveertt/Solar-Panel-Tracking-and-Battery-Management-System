#include <Wire.h>
#include <LiquidCrystal_AIP31068_I2C.h>
#include <Servo.h>
#include <math.h>

int thermistor_output1 = A3;
int thermistor_output2 = A2;

#define VOLTAGE_SENSOR_PIN A4
#define CURRENT_SENSOR_PIN A5

float adc_voltage = 0.0;
float in_voltage = 0.0;
float R1 = 30000.0;
float R2 = 7500.0;
float ref_voltage = 5.0;

int adc_value = 0;
float current_adc_voltage = 0.0;
float current = 0.0;
float current_sensor_sensitivity = 0.185;
float current_sensor_offset = 2.5;

int dustSensorPin = 10;
unsigned long duration;
unsigned long startTime;
unsigned long sampleTime = 1000;
float lowPulseOccupancy = 0;
float ratio = 0;
float concentration = 0;

Servo myservo;
int pos = 90;
int lastPos = 90;

int sample[5];
LiquidCrystal_AIP31068_I2C lcd(0x3E, 16, 2);  // I2C address 0x3E, 16x2 LCD

int relaypin = 7;
int relay = 6;

void setup() {
  Serial.begin(9600);
  pinMode(dustSensorPin, INPUT);
  startTime = millis();
  
  lcd.init();  // Initialize I2C LCD
  lcd.setCursor(0, 0);
  lcd.print("Enhanced Dual");
  lcd.setCursor(0, 1);
  lcd.print("Solar Panel");

  
  myservo.attach(9);
  myservo.write(pos);
  pinMode(relaypin, OUTPUT);
  pinMode(relay, OUTPUT);
  pinMode(relaypin,HIGH);
}

void loop() {
  int LDR1 = analogRead(A0);
  int LDR2 = analogRead(A1);

  // Voltage Measurement
  adc_value = analogRead(VOLTAGE_SENSOR_PIN);
  adc_voltage = ((adc_value * ref_voltage) / 1024.0);
  in_voltage = (adc_voltage * (R1 + R2) / R2);

  // Current Measurement
  adc_value = analogRead(CURRENT_SENSOR_PIN);
  current_adc_voltage = ((adc_value * ref_voltage) / 1024.0);
  current = -((current_adc_voltage - current_sensor_offset) / current_sensor_sensitivity);

  Serial.print("Input Voltage = ");
  Serial.println(in_voltage, 2);
  Serial.print("Current = ");
  Serial.println(current, 2);
  delay(500);

  // Temperature Measurement (Thermistor 1)
  int thermistor_adc_val;
  double output_voltage, thermistor_resistance, therm_res_ln;
  float temperature;
  thermistor_adc_val = analogRead(thermistor_output1);
  output_voltage = ((thermistor_adc_val * 5.0) / 1023.0);
  thermistor_resistance = ((5 * (10.0 / output_voltage)) - 10) * 1000;
  therm_res_ln = log(thermistor_resistance);
  temperature = (1 / (0.001129148 + (0.000234125 * therm_res_ln) + (0.0000000876741 * therm_res_ln * therm_res_ln * therm_res_ln))) - 273.15;
  Serial.print("Temperature1 in degree Celsius = ");
  Serial.println(temperature);
  delay(1000);

  // Temperature Measurement (Thermistor 2)
  int thermistor_adc_val2;
  double output_voltage2, thermistor_resistance2, therm_res_ln2;
  float temperature2;
  thermistor_adc_val2 = analogRead(thermistor_output2);
  output_voltage2 = ((thermistor_adc_val2 * 5.0) / 1023.0);
  thermistor_resistance2 = ((5 * (10.0 / output_voltage2)) - 10) * 1000;
  therm_res_ln2 = log(thermistor_resistance2);
  temperature2 = (1 / (0.001129148 + (0.000234125 * therm_res_ln2) + (0.0000000876741 * therm_res_ln2 * therm_res_ln2 * therm_res_ln2))) - 273.15;
  Serial.print("Temperature2 in degree Celsius = ");
  Serial.println(temperature2);
  delay(1000);

  // Dust Sensor Measurement
  duration = pulseIn(dustSensorPin, LOW);
  lowPulseOccupancy += duration;
  if (millis() - startTime >= sampleTime) {
    ratio = (lowPulseOccupancy / (sampleTime * 10.0)) * 100.0;
    concentration = 1.1 * ratio * ratio * ratio - 3.8 * ratio * ratio + 520 * ratio + 0.62;
    concentration = concentration / 1000000;
    Serial.print("Concentration: ");
    Serial.print(concentration);
    Serial.println(" g/m³");
    lowPulseOccupancy = 0;
    startTime = millis();
  }

  // LCD DISPLAY SEQUENTIALLY
  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("V=");
  lcd.print(in_voltage, 2);
  lcd.setCursor(0, 1);
  lcd.print("I=");
  lcd.print(current, 2);
  delay(2000);  // Show for 2 seconds

  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("T1=");
  lcd.print(temperature, 1);
  lcd.setCursor(0, 1);
  lcd.print("T2=");
  lcd.print(temperature2, 1);
  delay(2000);

  lcd.clear();
  lcd.setCursor(0, 0);
  lcd.print("Conc.=");
  lcd.print(concentration, 2);
  delay(2000);

  // Relay Control
  if ((in_voltage > 15) || (current > 1) || (temperature2 > 45)) {
    Serial.println("Relay OFF");
    digitalWrite(relay, HIGH);
  } else {
    Serial.println("Relay ON");
    digitalWrite(relay, LOW);
  }

  if ((concentration > 100) || (temperature > 40)) {
    digitalWrite(relaypin, LOW);
  } else {
    digitalWrite(relaypin, HIGH);
  }

  // Servo Motor Control
  if (LDR1 > LDR2 && LDR1 > 20) 
  {
    pos = min(lastPos + 5, 180);
  } 
  else if (LDR2 > LDR1 && LDR2 > 20) 
  {
    pos = max(lastPos - 5, 0);
  }

  if (pos != lastPos) 
  {
    myservo.write(pos);
    lastPos = pos;
  }

  Serial.print("\n");
  delay(10);
}
