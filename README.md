# MMA7455  
  
Arduino library for Freescale accelerometer MMA7455  
  
## Usage  
  
* init object  
MMA7455();  
  
* start object  
For non Arduino boards
void begin(uint8_t pin_sda, uint8_t pin_scl);  
  
For Arduino boards  
void begin();  

* read x-, y-, z- acceleration  
int xyz(uint16_t *pX, uint16_t *pY, uint16_t *pZ);  
  
* read 2 byte from sensor register (see MMA7455.h)  
int read(int start, uint8_t *buffer, int size);  
  
* write 2 byte to sensor register  
int write(int start, const uint8_t *pData, int size);  
  
