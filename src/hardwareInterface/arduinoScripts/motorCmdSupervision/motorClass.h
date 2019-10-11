#ifndef motorClass_h
#define motorClass_h


class motorClass{
public:
    // class
    motorClass(int pwmPin, int dirPin, int encPin, float gearRatio, float encCntsRev); 
      int _pwmPin;
      int _dirPin;
      int _encPin;
      float _gearRatio;
      float _encCntsRev;


    // set functions
    void setMotorPos(float pos);
      float desiredMotorPos = 0;
    void setMotorVel(float vel);
      float desiredMotorVel = 0;
    void setMotorForce(float force);
      float desiredMotorForce = 0;
    void setMotorForceD(float force);
      float desiredMotorForceD = 0;


    // encoder functions
    void clearEncoder(void);
    signed long readEnc(void);
      signed long encodercount = 0;


    // helper
    void storeOldVals(void);
      unsigned long prevTime = 0;
      signed long encodercountPrev = 0;
      float errorPosPrev = 0;
      float errorVelPrev = 0;
      float MotorForcePrev = 0; 
      float errorForcePrev = 0;
      float errorForceDPrev = 0;
    void calc_t(void);
      unsigned long currentTime = 0;
      float dt = 0;
    void stopMotor(void);
    void logValues(void);


    ////////////position\\\\\\\\\\\\\\\\\
    //Position controller gains
    float Kpp = 5000.0;
    float Kdp = 0.0;
    float Kip = 0;//100000;

    // position controller functions
    float motor_position_calc(void);
      float MotorPos;
    float pos_proportional_control(void);
      float errorPos = 0;
      float pCommandp = 0;
    float pos_derivative_control(void);
      float dCommandp = 0;
    float pos_integral_control(void);
      float integratedPosError = 0;
      float iCommandp = 0;
    int pos_closedLoopController(void);
      float currentCommandp = 0;


    ////////////velocity\\\\\\\\\\\\\\\\\
    //velocity controller gains
    float Kpv = 50;
    float Kdv = 0;
    float Kiv = 0.02;
        
    // velocity controller functions
    float motor_velocity_calc(void);
      float MotorVel = 0;
    float vel_proportional_control(void);
      float errorVel = 0;
      float pCommandv = 0;
    float vel_derivative_control(void);
      float dCommandv = 0;
    float vel_integral_control(void);
      float integratedVelError = 0;
      float iCommandv = 0;
    int vel_closedLoopController(void);
      float currentCommandv = 0;


    ////////////force\\\\\\\\\\\\\\\\\
    //force controller gains
    float Kpf = 150;
    float Kdf = 0;
    float Kif = 0.05;
  
    // force controller functions
    float motor_force_calc(void);
      float sentMotorForce = 0;
      float MotorForce = 0;
    float force_proportional_control(void);
      float errorForce = 0;
      float pCommandf = 0;
    float force_derivative_control(void);
      float dCommandf = 0;
    float force_integral_control(void);
      float integratedForceError = 0;
      float iCommandf = 0;
    int force_closedLoopController(void);
      float currentCommandf = 0;


    ////////////force derivative\\\\\\\\\\\\\\\\\
    //force derivative controller gains
    float Kpfd = 5;
    float Kdfd = 0;
    float Kifd = 0.002;
        
    // force derivative controller functions
    float motor_force_d_calc(void);
      float MotorForceD = 0;
    float force_d_proportional_control(void);
      float errorForceD = 0;
      float pCommandfd = 0;
    float force_d_derivative_control(void);
      float dCommandfd = 0;
    float force_d_integral_control(void);
      float integratedForceDError = 0;
      float iCommandfd = 0;
    int force_d_closedLoopController(void);
      float currentCommandfd = 0;
};


//Constants
const float MAX_PWM = 1000;
const float MIN_PWM = -1000;
const float on_off_tolerance = 0.07;

#endif
