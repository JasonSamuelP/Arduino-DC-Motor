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

// Initial Set up for Encoder
const int encoderPinA = 3;    // Pin 3 interrupt in Arduino will receive encoder value from channel A
const int encoderPinB = 2;    // Pin 2 interrupt in Arduino will receive encoder value from channel B
volatile long encoderPos = 0; // Encoder value will be set as encoderPos variable and will be set as 0 for initial value
                              // Volatile means that this variable can change any time during the interrupt service routine
long previousMillis = 0;      // the previous count value was initialized to zero
long currentMillis = 0;       // the current count value was initialized to zero
int interval = 1000;          // The interval is set to 1000 millisecond / 1 second
                              // since the program was set to calculate rpm and clears encoder pulse counter every 1 second
float rpm = 0;                  // rpm value or the actual speed was initialized to 0
#define ENCODEROUTPUT 650     // the encoder output was set to 650 value since every turn of the motor the encoder outputs 663 times.
                              // This value was obtained by using trial and error since the exact Count per revolute was not given in this lab
void doEncoderA()             // Function to calculate the encoder Value
{  
  encoderPos += (digitalRead(encoderPinA)==digitalRead(encoderPinB))?-1:1; // using ternary operator
  // encoderPos = -1 + encoderPos if true
  // encoderPos = 1 + enncoderPos If false
}
void doEncoderB()
{  
  encoderPos += (digitalRead(encoderPinA)==digitalRead(encoderPinB))?1:-1; // using ternary operator
  // encoderPos = 1 + encoderPos if true
  // encoderPos = -1 + enncoderPos If false
}

// Initial Set up for Controller
float Kp = 0.71;      // Kp or proportional constant was set to 0.71 for this experiment, it was obtained using trial and error
float control = 0;    // The control value is the controller output for the proportional controller
float error = 0;      // The error was defined as 0 initially
float PWM;            // PWM value was set as float datatype, it is used to send PWM value to the enable pin


void setup() 
{  
  Serial.begin(9600);// setup Serial Monitor to display information
  
  // Encoder and Interrupt
  pinMode(encoderPinA, INPUT_PULLUP);        // EncoderPinA was set as input_pullup, which the pin is effectively connected through an internal resistor to the VCC+ power line
  attachInterrupt(0, doEncoderA, RISING);    // Interrupt number 0 means for Arduino pin number 2, it will cal the doencoderA function to calculate the encoder value
                                             // CHANGED is used to trigger the interrupt whenever the pin change the value 
  pinMode(encoderPinB, INPUT_PULLUP);        //  EncoderPinA was set as input_pullup, which the pin is effectively connected through an internal resistor to the VCC+ power line
  attachInterrupt(1, doEncoderB, FALLING);    // Interrupt number 1 means for Arduino pin number 3,
                                             // CHANGED is used to trigger the interrupt whenever the pin change the value
  previousMillis = millis();                 // The previous count value was started to count
                                             // millis function in here will return the current value of the arduino internal timer in milisecond

  // General Setup for other PINS
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
  
// Generating desired speed (speedz) from potentiometer
float sensorValue = analogRead(Potentio);         // the analog read function will read the potentio value and stored it in sensorValue variable
float speedz;                                     // speedz was declared and has float datatype
speedz = map(sensorValue, 0, 1023, 0, 265);       // the map function will convert the min and max value of the potentiometer (0-1023) to max and min speed value (0-265)

// Proportional Control
error = speedz - abs(rpm);        // error is obtained from desired speed - actual speed
control = Kp*error + speedz;      // In this case the bias was not constant since the desired speed is always changing according to potentiometer rotation values 
                                  // Hence, bias was set as speedz variable
PWM = constrain(control, 0, 255); // The control value will be converted to pwm value where  0 is the minimum value and 255 is the maximum value
analogWrite(EN1, PWM);            // The current PWM value will be sent to enable pin 1 and this pwm value will be updated for each iteration by proportional control
  
// RPM calculation and it will be updated for every second
  currentMillis = millis();                      // The previous count value was started to count
  if (currentMillis - previousMillis > interval) // it will update for every 1 second
  {
    previousMillis = currentMillis; // The current count value was set to previous count value after 1 second
    Serial.print("Encoder ");
    Serial.print(encoderPos); // Display the encoder value
    Serial.print("| PWM ");
    Serial.print(PWM);        // Display the PWM value
    // Calculating RPM
    rpm = abs((encoderPos * 60 / ENCODEROUTPUT)); // abs is used so the RPM value is not negative when the motor is rotating counterclockwise or clockwise direction
    Serial.print("| speedz "); 
    Serial.print(speedz);     // Display the Desired speed value
    Serial.print("| RPM ");
    Serial.print(rpm);        // Display the RPM value
    Serial.print("| error (speedz - Previous RPM) "); 
    Serial.println(error);    // Display the error value
     encoderPos = 0;      // The encoder value will be set back to 0
  }
  
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
   
  delay(50);// 50 millisecond delay  
}//loop end
