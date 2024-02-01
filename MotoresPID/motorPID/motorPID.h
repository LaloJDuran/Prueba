#ifndef MOTORPID_H
#define MOTORPID_H

#include "Arduino.h"

class motorPID{
    public:
        motorPID(double, double, double, unsigned long, double, double, double, double);
		motorPID(unsigned long);

		double scalePWM(double);
		double scaleVel(double);

        double PID(double, double);

        bool reset();

        void setGains(double, double, double);

        void setSampleTime(unsigned long);

        void setPWMLimits(double, double);
		
		void setVelLimits(double, double);
		
		double getKp();						  
	    double getTi();						
	    double getTd();				
    
	private:
        double _Kp;            // Proportional gain
        double _Ti;            // Integral time gain
        double _Td;            // Derivative gain
        double _kp;             
    	double _ti;             
    	double _td;            

    	double state[3];       // State array of length 4 to store e[k-1], e[k-2], u[k-1] and u[k-2]
	    double Ts;             // Sample time
        double _PWMMax;     // Maximum output value of the controller
        double _PWMMin;     // Minimum output value of the controller
		double _velMax;     // Maximum input value of the controller
        double _velMin;     // Minimum input value of the controller
}; 

#endif