class Encoder
{
  public:
    Encoder(int A,int B, int Z);
    void setup(); // Setup hardware
    int readEncoder(); // read encoder
    bool status(); // gives direction 
    
  private:
    int _A; // A Channel
    int _B; // B Channel
    int _Z; // Z Pulse
    int _Counter;
};

Encoder::Encoder(int A,int B, int Z)
{
  _A = A;
  _B = B;
  _Z = Z;
  
}

void Encoder::setup()
{
  // Setup Inputs
  pinMode(_A,INPUT_PULLUP);
  pinMode(_B,INPUT_PULLUP);
  pinMode(_Z,INPUT_PULLUP);

}

// Method to start motion in clockwise direction
int Encoder::readEncoder()
{
  
  
  return _Counter;
}




