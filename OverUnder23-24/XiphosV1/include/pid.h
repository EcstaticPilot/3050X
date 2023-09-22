
class PID{
    protected:
    float kp,ki,kd;
    float error, prevError, totalError;
    float output;
    public:
    PID(float kp, float ki, float kd, float error):kp(kp),ki(ki),kd(kd),error(error){
    };
    
    void update(float input){
        error = input;
        output = kp*error+ki*totalError+kd*(prevError-error);
        totalError +=error;
        prevError=error;
    }
};

