#pragma once
class PID{
    public:
    float error;
    float prevError, totalError;

    float kp,ki,kd;
    float output;
    float intergalLimit = 100;

    PID(float kp, float ki, float kd, float error, float integralLimit = 100):kp(kp),ki(ki),kd(kd),error(error),intergalLimit(intergalLimit){
        prevError=error;
        totalError=0;
    };
    
    /**
     * @brief updates the PID loop
     * 
     */
    void update();

    /**
     * @brief prints out the pid values in the order of error, kp, ki, kd, outputss
     * 
     */
    void print(float error);
};

