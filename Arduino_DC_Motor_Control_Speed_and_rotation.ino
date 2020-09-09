#define P1A 10          // Pin 10 is defined as P1A (L293D input pin, pin number 2)         
#define P2A 11          // Pin 11 is defined as P2A (L293D input pin, pin number 7)     
#define EN1 9           // Pin 9 is defined as EN1 (L293D enable pin, pin number 1)     
#define PushButton 12   // Pin 12 is defined as push button pin 
#define BlueLed 13      // Pin 13 is defined for Blue LED 
#define RedLed 8        // Pin 8 is defined for Red LED

// Initial set up for potentiometer 
double Potentio = A0;   // Potentio variable is defined as A0 pin with double datatype (define the potentiometer)
int sensorValue = 0;    // sensorValue is set as integer (The analogue reading from potentiometer)
// Initial Set up for push button mechanism
bool pressed = false;   // pressed is set "false" initially 
int rotDirection = 0;   // rotDirection variable is set to 0


void setup() 
{
  Serial.begin(9600);         // Initialize For serial monitor 
  pinMode(P1A, OUTPUT);       // P1A is defined as OUTPUT
  pinMode(P2A, OUTPUT);       // P2A is defined as OUTPUT
  pinMode(EN1, OUTPUT);       // EN1 is defined as OUTPUT
  pinMode(PushButton, INPUT); // PushButton is defined as INPUT
  pinMode(BlueLed, OUTPUT);   // Blue LED is defined as OUTPUT
  pinMode(RedLed, OUTPUT);    // Red LED is defined as OUTPUT
  pinMode(Potentio, INPUT);   // Potentio is defined as INPUT
   
  // Initial Rotation direction --- CLOCKWISE --> 
  // The initial configuration is set as clockwise hence P1A and P2A are Low and high respectively and 
  // RedLED and Blue LED are LOW
  digitalWrite(P1A, LOW);
  digitalWrite(P2A, HIGH);
  digitalWrite(RedLed,LOW);
  digitalWrite(BlueLed,LOW);
  Serial.println("Arduino DC Motor Control"); // Print on the serial monitor
}

void loop() 
{
// Potentiometer
sensorValue = analogRead(Potentio);         // the analog read function will read the potentio value and stored it in sensorValue variable
double speedz = 0;                          // speedz was declared as 0 and has double datatype
speedz = map(sensorValue, 0, 1023, 0, 255); // the map function will convert the min and max value of the potentiometer (0-1023) to PWM value (0-255)
// For the PWM Signal ,  0 means signal of 0% duty cycle and 255 means signal of 100% duty cycle
analogWrite(EN1, speedz);                   // the analogWrite will send the pwm signal in speedz variable to EN1 pin


float RPM;                                  // RPM was introduced to display the motor speed
RPM = map(speedz,0,255, 0, 265);            // Where 0 rpm is the min speed and 265 rpm is the max speed
Serial.println(RPM);                     // RPM is printed on the serial monitor

// Button mechanism
if (digitalRead(PushButton) == true)        // if we press the push button, pressed will have "true" value since we declared it as "false"
  {
    pressed = !pressed;
  }
while (digitalRead(PushButton) == true);    // This function enable us to push the button at period of time
                                            // while the push button is true (being pressed), it will do nothing

// Once we release the pushbutton . . .     
                                 
// Change rotation direction to CCW
if (pressed == true  & rotDirection == 0)   // at this case, pressed has became "true" and rotDirection is initialized as 0 value in the beginning
  {
    digitalWrite(P1A, HIGH);                // P1A became HIGH
    digitalWrite(P2A, LOW);                 // P2A became LOW
    digitalWrite(RedLed,HIGH);              // Turn on RedLed
    digitalWrite(BlueLed,LOW);              // Turn off BlueLED
    rotDirection = 1;                       // rotDirection is set to 1
    
  }

// Once we press again the pushbutton and release it, it will go through the first two function button mechanism and pressed value became false and rotdirection is 1 
// Change rotation direction to CW
if (pressed == false & rotDirection == 1)   // at this case, pressed has became "false" and rotDirection is 1
  {
    digitalWrite(P1A, LOW);                 // P1A became LOW
    digitalWrite(P2A, HIGH);                // P2A became HIGH
    digitalWrite(RedLed,LOW);               // Turn off RedLEd
    digitalWrite(BlueLed,HIGH);             // Turn On BlueLED
    rotDirection = 0;                       //rotDirection is set to 0
  }
    delay(75);                              // 75 millisecond delay
}
