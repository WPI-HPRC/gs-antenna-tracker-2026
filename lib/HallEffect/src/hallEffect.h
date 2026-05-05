class HallEffect
{
public:
    HallEffect(int pin);
    void initializeHallEffect(void);
    bool readSensor(void);
    
protected:
    int pinNum;
};
