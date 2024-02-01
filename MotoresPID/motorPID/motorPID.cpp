#include "motorPID.h"

motorPID::motorPID(double Kp, double Ti, double Td, unsigned long sampleTime, double velMax, double velMin, double pwmMax, double pwmMin){
    setSampleTime(sampleTime);
    setGains(Kp, Ti, Td);
    setPWMLimits(pwmMax,pwmMin);   
	setVelLimits(velMax,velMin);
    
    memset(state, 0, 3u * sizeof(double));  //Asignar 3 espacios de memoria para state (errores)
}


motorPID::motorPID(unsigned long sampleTime) :motorPID(0.0,0.0,0.0,sampleTime,0.0,0.0,0.0,0.0)
{
}

// Recive Velocidad en rad/s
double motorPID::PID(double setpoint,double input) {
	
    // Transforma en %
	setpoint = scaleVel(setpoint);
	input = scaleVel(input);

    double error = setpoint - input;
		
	state[2] += Ts*error; // Termino integral
	state[1] = (error-state[0])/Ts; // Termino derivativo
	state[0] = error; // error[k-1]
	
	double output = _kp*(error+(1/_ti)*state[2]+_td*state[1]);
   
    /* Anti-windup */
    if(output > 100) {
        output = 100;
    }else if(output < -100) {
        output = -100;
    }
		
    // Aqui da el porcentaje del PWM al que se debe de mover
    // Por eso se escala 
    output = scalePWM(output);
    return output;
}   

double motorPID::scalePWM(double pwm) 
{
  if (pwm >= 0) return (pwm) * (_PWMMax - _PWMMin) / (100.0) + _PWMMin; else return (pwm) * (-_PWMMax + _PWMMin) / (-100.0) - _PWMMin;
}

double motorPID::scaleVel(double vel) 
{
	return vel*(100.0/_velMax);
}

bool motorPID::reset() {
    /* Clear the state buffer. The size will be always 3 samples */
    memset(state, 0, 3u*sizeof(double));
    return true;
}

void motorPID::setGains(double Kp, double Ti, double Td) {
	_kp = Kp;
	_ti = Ti;
	_td = Td;
}

double motorPID::getKp(){ return  _kp;}
double motorPID::getTi(){ return  _ti;}
double motorPID::getTd(){ return  _td;}

void motorPID::setSampleTime(unsigned long sampleTime) {
    /* Sample time in ms to seconds */
    Ts = (double)sampleTime/1000.0;
}

void motorPID::setPWMLimits(double pwmMax, double pwmMin) {
	_PWMMax = pwmMax;
    _PWMMin = pwmMin;
}

void motorPID::setVelLimits(double velMax, double velMin) {
    _velMax = velMax;
    _velMin = velMin;
}
