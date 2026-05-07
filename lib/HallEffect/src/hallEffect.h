#include <Arduino.h>

class HallEffect
{
public:
    HallEffect(uint8_t pin);
    void initializeHallEffect(void);
    bool readSensor(void);
    
protected:
    uint8_t pinNum;
    uint8_t arrSize = 5;
    bool prevReadings[5] = {0};
    bool recentState = false;
    uint8_t readingCount = 0;
};