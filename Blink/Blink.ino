#define LED_Blue 4
#define LED_red 2
void setup() {
  // initialize digital pin LED_BUILTSIN as an output.
  Serial.begin(115200);
  pinMode(LED_Blue, OUTPUT);
  pinMode(LED_red, OUTPUT);
}

// the loop function runs over and over again forever
void loop() {
  digitalWrite(LED_Blue, HIGH);
  digitalWrite(LED_red, HIGH);
  Serial.println("turn on");
  delay(1000);
  digitalWrite(LED_red, LOW);
  digitalWrite(LED_Blue, LOW);
  delay(1000);
  Serial.println("turn off");
}
