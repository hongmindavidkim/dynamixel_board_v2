#ifndef FORCESENSOR_H
#define FORCESENSOR_H

#include "neural_nets.h"
#include "mbed.h"

extern int32_t pressure_raw1[8];
extern int32_t pressure_raw2[8];

class ForceSensor{
public:
    ForceSensor(int channel_number, NeuralNet *neural_net);
    void Sample();
    void Initialize();
    void Calibrate();
    void Evaluate();
    
    int _channel;
    int raw_data[8];
    int offsets[8];
    int offset_data[8];
    float input_data[8];
    float output_data[5];
    
    
private:
    void config_dev(struct bmp3_dev *dev, int channel);
    NeuralNet *_net;
    uint8_t sensor_comp;
    
    
    float l1[12]; // to be evaluated on-line
    float l2[64]; //[25];
    float l3[64]; //[5];
    float l4[5];

};

#endif
