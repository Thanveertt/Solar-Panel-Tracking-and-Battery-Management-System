// Pin configuration
const int dustSensorPin = 10;  // Connect to Vout 2 of DSM501A
unsigned long duration;       // To measure LOW pulse duration
unsigned long startTime;      // To calculate sampling period
unsigned long sampleTime = 3000; // Sampling time in milliseconds (5 seconds)
float lowPulseOccupancy = 0;  // Sum of LOW pulse durations
float ratio = 0;              // LOW pulse ratio
float concentration = 0;      // Dust concentration in mg/m3

void setup() {
  pinMode(dustSensorPin, INPUT);
  Serial.begin(9600);         // Initialize serial communication for output
  startTime = millis();       // Record the starting time
}

void loop() {
  duration = pulseIn(dustSensorPin, LOW);  // Measure LOW pulse width
  lowPulseOccupancy += duration;

  if (millis() - startTime >= sampleTime) {
    // Calculate the LOW pulse ratio
    ratio = (lowPulseOccupancy / (sampleTime * 10.0)) * 100.0;

    // Convert ratio to concentration in mg/m3
    // Formula derived from sensor characteristics
    concentration = 1.1 * ratio * ratio * ratio - 3.8 * ratio * ratio + 520 * ratio + 0.62;
    concentration = concentration / 1000000;  // Now in g/m³


    Serial.print("Concentration: ");
    Serial.print(concentration);
    Serial.println(" g/m³");
    

    // Reset for the next measurement cycle
    lowPulseOccupancy = 0;
    startTime = millis();
  }
}
