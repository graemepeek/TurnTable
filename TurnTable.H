class TurnTable
{
  public:
    TurnTable(int stepPin,int directionPin, int clockWisePin, int counterClockWisePin);
    void setup(); // Setup hardware
    void startMotionCW(int stepperSpeed, int speedMultiplier); // Starts motion in clockwise direction
    void startMotionCCW(int stepperSpeed, int speedMultiplier); // starts motion in counter clockwise direction
    void stopMotion(); // Stops all motion
    bool status(); // future implementation  
    
  private:
    int _stepPin; // Pulse train signal out to stepper drive
    int _directionPin; // Direction signal out to stepper drive
    int _clockWisePin; // Input to enable clock wise motion in mode 0
    int _counterClockWisePin; // Input to enable counter clockwise motion in mode 
};

TurnTable::TurnTable(int stepPin,int directionPin, int clockWisePin, int counterClockWisePin)
{
  _stepPin = stepPin;
  _directionPin = directionPin;
  _clockWisePin = clockWisePin;
  _counterClockWisePin = counterClockWisePin;
}

void TurnTable::setup()
{
  // Setup outputs
  pinMode(_stepPin,OUTPUT);
  pinMode(_directionPin,OUTPUT);
  // Setup inputs
  pinMode(_clockWisePin, INPUT_PULLUP);
  pinMode(_counterClockWisePin, INPUT_PULLUP);
}

// Method to start motion in clockwise direction
void TurnTable::startMotionCW(int stepperSpeed, int speedMultiplier)
{
  int stepDelay = stepperSpeed * speedMultiplier;
  digitalWrite(_directionPin, LOW); // Turn off direction pin to go clockwise
  digitalWrite(_stepPin, HIGH);
  delayMicroseconds(stepDelay);
  //delay(Stepdelay); // Approximately 10% duty cycle @ 1KHz
  digitalWrite(_stepPin, LOW);
  delayMicroseconds(stepDelay);
  //delay(Stepdelay);
  #ifdef DEBUG_BUILD 
    Serial.println("CW");
  #endif
}

// This routine makes the turntable go counter clockwise
void TurnTable::startMotionCCW(int stepperSpeed, int speedMultiplier){
  int stepDelay = stepperSpeed * speedMultiplier;
  digitalWrite(_directionPin, HIGH);
  digitalWrite(_stepPin, HIGH);
  delayMicroseconds(stepDelay);
  //delay(Stepdelay); // Approximately 10% duty cycle @ 1KHz
  digitalWrite(_stepPin, LOW);
  //delayMicroseconds(Stepdelay);
  delay(stepDelay);
  #ifdef DEBUG_BUILD 
    Serial.println("CCW");
  #endif
}

// Method to stop motion
void TurnTable::stopMotion(){
  digitalWrite(_directionPin, LOW);
  digitalWrite(_stepPin, LOW);
  #ifdef DEBUG_BUILD 
    Serial.print("Stop");
  #endif
}


