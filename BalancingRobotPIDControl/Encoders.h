#ifndef _encoder_h_
#define _encoder_h_

class Encoder {
public:
    Encoder(PinName pinA, PinName pinB) : _HallSensorA(pinA), _HallSensorB(pinB) {
        _counter = 0;
        _HallSensorA.rise(this, &Encoder::EncodeA);
        _HallSensorA.mode(PullDown);
        //_HallSensorA.fall(this, &Encoder::EncodeA);
        //_HallSensorB.rise(this, &Encoder::EncodeB);
        //_HallSensorB.fall(this, &Encoder::EncodeB);
    }
    int read() {
        return _counter;
    }
    void reset(long targetPosition) {
        _counter = targetPosition;
    }
private:
    volatile long _counter;
    InterruptIn _HallSensorA;
    /*InterruptIn*/ DigitalIn _HallSensorB;
    
    void EncodeA() {
        if (_HallSensorA.read() == _HallSensorB.read())
            _counter++;
        else
            _counter--;
    }
    /*
     void EncodeB() {
     if (_HallSensorA.read() == _HallSensorB.read())
     _counter--;
     else
     _counter++;
     }
     */
};

#endif