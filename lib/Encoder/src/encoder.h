class Encoder
{
public:
    Encoder(int aPin, int bPin);

    void InitializeEncoder(void);
    int ReadPosition(void);
    int ReadVelocity(void);

protected:
    int aPin;
    int bPin;

    int count;
};