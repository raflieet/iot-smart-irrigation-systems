#define echoPin 18 // attach pin GPIO18 ESP32 to pin Echo of JSN-SR04
#define trigPin 5 // attach pin GPIO5 ESP32 to pin Trig of JSN-SR04                     

long duration; // Time taken for the pulse to reach the receiver
int distance; 

void setup()
{
  pinMode(trigPin, OUTPUT); 
  pinMode(echoPin, INPUT);  
  Serial.begin(9600);
  Serial.println("Distance measurement using JSN-SR04T");
  delay(500);
}

void loop()
{
  digitalWrite(trigPin, LOW);
  delayMicroseconds(2); 

  digitalWrite(trigPin, HIGH); // turn on the Trigger to generate pulse
  delayMicroseconds(10); // keep the trigger "ON" for 10 ms to generate pulse
  digitalWrite(trigPin, LOW); // Turn off the pulse trigger to stop pulse

  // If pulse reached the receiver echoPin
  // become high Then pulseIn() returns the
  // time taken by the pulse to reach the receiver
  duration = pulseIn(echoPin, HIGH);
  distance = duration * 0.0344 / 2; 

  Serial.print("Distance: ");
  Serial.print(distance);
  Serial.println(" cm");
  delay(100);
}
