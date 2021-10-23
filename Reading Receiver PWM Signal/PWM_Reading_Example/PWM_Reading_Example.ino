float channel;

void setup() {
  // put your setup code here, to run once:
  pinMode(2,INPUT);
  Serial.begin(9600);
}

void loop() {
  channel = pulseIn(2, HIGH);
  Serial.println(channel);
}
