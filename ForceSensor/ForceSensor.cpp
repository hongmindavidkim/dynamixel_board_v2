
#include "mbed.h"
#include "ForceSensor.h"
#include "../math_ops.h"
//#include <math.h>

ForceSensor::ForceSensor(int channel_number, NeuralNet *neural_net){
    
    // store channel number and pointer for neural net struct
    // TODO: implement check that channel is 1, 2, or 3
    _channel = channel_number;
    _net = neural_net;
    
    // sensors are structs are already initialized
    
    // initialize other values
    sensor_comp = uint8_t(1); //| uint8_t(1<<1); // sensor_comp = BMP3_PRESS | BMP3_TEMP;
    
    }
    
void ForceSensor::Initialize(){     
    
}
    
    
void ForceSensor::config_dev(struct bmp3_dev *dev, int channel){
    
}
    
void ForceSensor::Sample(){
    
    // get data from every sensor
    if (_channel == 0){
        for(int i = 0;i<8;i++){
            raw_data[i] = pressure_raw1[i];
        }
    }
    if (_channel == 1){
        for(int i = 0;i<8;i++){
            raw_data[i] = pressure_raw2[i];
        }
    }
    
    // could combine this with previous step
    offset_data[0] = raw_data[0]-offsets[0];
    offset_data[1] = raw_data[1]-offsets[1];
    offset_data[2] = raw_data[2]-offsets[2];
    offset_data[3] = raw_data[3]-offsets[3];
    offset_data[4] = raw_data[4]-offsets[4];
    offset_data[5] = raw_data[5]-offsets[5];
    offset_data[6] = raw_data[6]-offsets[6];
    offset_data[7] = raw_data[7]-offsets[7];
    
    
}
    
void ForceSensor::Evaluate(){
    // scales raw input data, evaluates neural network, scales and stores output data
        
    // scale sensor data
    for (int i=0; i<8; i++){
        input_data[i] = 0.0f;
        input_data[i] = (((float)offset_data[i]) - (_net->minims[i+5]))/(_net->maxims[i+5]-_net->minims[i+5]); // / _net->max_pressure;
        // check that inputs are between 0 and 1?
    }

    // decode sensor data here....521*4 operations (multiply,add,activation,add)
    // reset values
    for (int i = 0; i<12; i++){
        l1[i] = 0.0f;
    }
    for (int i = 0; i<64; i++){ //i<25
        l2[i] = 0.0f;
        l3[i] = 0.0f;
    }
    for (int i = 0; i<5; i++){
        l4[i] = 0.0f;
    }
        
    // layer 1
    for(int i = 0; i<12; i++){ // for each node in the next layer
        for(int j = 0; j<8; j++){ // add contribution of node in prev. layer
            l1[i] +=  (_net->w1[j][i]*input_data[j]); 
        }
        l1[i] += _net->b1[i]; // add bias
        l1[i] = fmaxf(0.0f, l1[i]); // relu activation
    }
        
    // layer 2
    for(int i = 0; i<64; i++){ // for each node in the next layer
        for(int j = 0; j<12; j++){ // add contribution of node in prev. layer
            l2[i] += (_net->w2[j][i]*l1[j]);
        }
        l2[i] += _net->b2[i]; // add bias
        l2[i] = fmaxf(0.0f, l2[i]); // relu activation
    }   
    
    // layer 3 // added for larger network architecture
    for(int i = 0; i<64; i++){ // for each node in the next layer
        for(int j = 0; j<64; j++){ // add contribution of node in prev. layer
            l3[i] += (_net->w3[j][i]*l2[j]);
        }
        l3[i] += _net->b3[i]; // add bias
        l3[i] = fmaxf(0.0f, l3[i]); // relu activation
    }   
    
    // layer 4
    for(int i = 0; i<5; i++){ // for each node in the next layer
        for(int j = 0; j<64; j++){ // add contribution of node in prev. layer
            l4[i] += _net->w4[j][i]*l3[j]; 
        }
        l4[i] += _net->b4[i];// add bias
        l4[i] = fmaxf(0.0f, l4[i]); // relu activation
    }  

    // post-process, re-scale decoded data
    for (int i=0; i<5; i++) {
        output_data[i] = 0.0f;
        output_data[i] = (l4[i]*(_net->maxims[i]-_net->minims[i])) + _net->minims[i]; // - abs(_net->minims[i]);
    
    }      
    
}
    
void ForceSensor::Calibrate(){
        
    printf("Calculating sensor offsets.\n\r");
    float temp_offsets[] = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f};
    int num_samples = 10;
    for (int i=0; i<num_samples; i++){
        Sample();
        for (int j=0; j<8; j++){
            temp_offsets[j] += ((float)raw_data[j])/((float)num_samples);
        }
        wait_us(10000); // wait for 1/200 s for next sample
    }
    printf("Saved offsets: ");
    for (int i=0; i<8; i++){
        offsets[i] = (int)temp_offsets[i];
        if (i<7) {
            printf("%d, ", offsets[i]);
        } else {
            printf("%d\r\n", offsets[i]);
        }
    }
    wait_us(100000); // wait for 1/200 s for next sample
    
} 
    
    




    





//void calibrateSensor(uint16_t* offsets){
//    
//    float temp_offsets[8] = {0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f, 0.0f};
//    int samps = 10;
//    
//    for (int i=0; i<samps; i++){
//        for (int j=0; j<8; j++){
//            temp_offsets[j] += (float)spi3.binary(j);
//        }
//        wait_ms(1);
//    }
//    
//    for (int i=0; i<8; i++){
//        temp_offsets[i] = temp_offsets[i]/((float)samps); // get overall offset
//        offsets[i] = (uint16_t)temp_offsets[i]; // convert to int
//    }
//
//    }