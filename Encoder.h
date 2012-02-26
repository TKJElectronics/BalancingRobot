#ifndef _encoder_h_
#define _encoder_h_

class Encoder {
public:
    Encoder(PinName pinA, PinName pinB) : _HallSensorA(pinA), _HallSensorB(pinB) {
        _counter = 0;
        _HallSensorA.rise(this, &Encoder::EncodeA);
    }
    long read() {
        return _counter;
    }
private:
    volatile long _counter;
    InterruptIn _HallSensorA;
    DigitalIn _HallSensorB;
    
    void EncodeA() {
        if (_HallSensorB.read())
            _counter++;
        else
            _counter--;
    }
};

#endif