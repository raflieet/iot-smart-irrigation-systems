
volatile int flow_frequency; // Measures flow sensor pulses

// Calculated litres/hour
float vol = 0.0, l_minute;
unsigned char flowsensor = 34; // Sensor Input
unsigned long currentTime;
unsigned long cloopTime;
float volume_per_pulse = 0.0;

// Interrupt function
void flow (){
  flow_frequency++;
}

// Setup Function
void setup() {
  Serial.begin(115200);

  pinMode(flowsensor, INPUT);
  digitalWrite(flowsensor, HIGH); // Optional Internal Pull-Up
  attachInterrupt(digitalPinToInterrupt(flowsensor), flow, RISING); // Setup Interrupt
  currentTime = millis();
  cloopTime = currentTime;
  // Calibrate and set the value of volume_per_pulse here
  volume_per_pulse = 0.004; // Replace with the actual value you've measured

}

// Loop Function
void loop() {
  currentTime = millis();
  // Every second, calculate and print litres/hour
  if (currentTime >= (cloopTime + 1000)){
    Serial.println(currentTime);
    cloopTime = currentTime; // Updates cloopTime
    if (flow_frequency != 0){
      // Pulse frequency (Hz) = 7.5Q, Q is flow rate in L/min.
      //      l_minute = (flow_frequency / 7.5); // (Pulse frequency x 60 min) / 7.5Q = flowrate in L/hour
      l_minute = (flow_frequency * volume_per_pulse * 60);
      Serial.print("Rate: ");
      Serial.print(l_minute);
      Serial.println(" L/M");

      vol = vol + l_minute / 60;
      Serial.print("Vol:");
      Serial.print(vol);
      Serial.println(" L, ");
      Serial.print("Vol:");
      Serial.print(vol * 1000);
      Serial.println(" mL");

      flow_frequency = 0; // Reset Counter
    }
    else{
      Serial.println("Rate:0 L/M ");
    }
  }
  delay(10);
}
