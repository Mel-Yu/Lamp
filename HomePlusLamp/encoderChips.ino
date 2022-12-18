
void initLS7366(int _ssPin)
{
  
  // Set slave selects as outputs
  pinMode(_ssPin, OUTPUT);
  
  // Raise select pins
  // Communication begins when you drop the individual select signsl
  digitalWrite(_ssPin, HIGH);

  // Initialize encoder
  //    Clock division factor: 0
  //    Negative index input
  //    free-running count mode
  //    x4 quatrature count mode (four counts per quadrature cycle)
  // NOTE: For more information on commands, see datasheet
  digitalWrite(_ssPin, LOW);        // Begin SPI conversation
  SPI.transfer(0x88);                       // Write to MDR0
  SPI.transfer(0x03);                       // Configure to 4 byte mode
  digitalWrite(_ssPin, HIGH);       // Terminate SPI conversation
  
}

int readEncoder(int _ssPin)
{

  // Initialize temporary variables for SPI read
  int32_t count_1, count_2, count_3, count_4;
  int32_t cnt;  
  
  digitalWrite(_ssPin, LOW);      // Begin SPI conversation
  SPI.transfer(0x60);                     // Request count
  count_1 = SPI.transfer(0x00);           // Read highest order byte
  count_2 = SPI.transfer(0x00);           
  count_3 = SPI.transfer(0x00);           
  count_4 = SPI.transfer(0x00);           // Read lowest order byte
  digitalWrite(_ssPin, HIGH);     // Terminate SPI conversation 
    
  // Calculate encoder count
  cnt = (count_1 << 8) + count_2;
  cnt = (cnt << 8) + count_3;
  cnt = (cnt << 8) + count_4;
  
  return cnt;
}


void clearEncoder(int _ssPin)
{
    // Set encoder's data register to 0
  digitalWrite(_ssPin, LOW);      // Begin SPI conversation  
  // Write to DTR
  SPI.transfer(0x98);    
  // Load data
  SPI.transfer(0x00);  // Highest order byte
  SPI.transfer(0x00);           
  SPI.transfer(0x00);           
  SPI.transfer(0x00);  // lowest order byte
  digitalWrite(_ssPin, HIGH);     // Terminate SPI conversation 

  delayMicroseconds(100);  // provides some breathing room between SPI conversations
  
  // Set encoder current data register to center
  digitalWrite(_ssPin, LOW);      // Begin SPI conversation  
  SPI.transfer(0xE0);    
  digitalWrite(_ssPin, HIGH);     // Terminate SPI conversation
}
