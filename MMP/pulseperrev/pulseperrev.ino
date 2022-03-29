#define encoder1 18

volatile long pulsecount = 0;

void setup()
{
    Serial.begin(9600);

    pinMode(encoder1,INPUT_PULLUP);
    attachInterrupt(digitalPinToInterrupt(encoder1), pulse, RISING);
  }

void loop() {
  
    Serial.print(" Pulses: ");
    Serial.println(pulsecount);  
}
 
// Increment the number of pulses by 1
void pulse() {
  pulsecount++;
}
