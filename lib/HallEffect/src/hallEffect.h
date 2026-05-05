#include <Arduino.h>

class HallEffect
{
public:
    HallEffect(uint8_t pin);
    void initializeHallEffect(void);
    bool readSensor(void);
    
protected:
    uint8_t pinNum;
};