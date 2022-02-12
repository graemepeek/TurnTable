class Potentiometer
{
  public:
    Potentiometer(int channel, int position);
    void setup(); // Setup hardware
    int readPotentiometer(); // read value
    
  private:
    int _channel;
    int _positionIn;
    int _positionOut;
};

Potentiometer::Potentiometer(int channel,int position)
{
  _channel    = channel;
  _positionIn = position;
    
}

void Potentiometer::setup()
{
  // Setup 
  

}

// Method to read Potentiometer
int Potentiometer::readPotentiometer()
{
  
  //map(value, fromLow, fromHigh, toLow, toHigh)
  _positionOut =  map(analogRead(_channel), 0, 1023, 1, 360);
  return _positionOut;
}




