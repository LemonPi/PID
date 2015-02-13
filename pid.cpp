#include "Arduino.h"
#include "pid.h"

template <typename T>
PID<T>::PID(volatile const T& in, const T& set, T& out, double p, double i, double d, bool res)
		: 
		input(&in), 
		setpoint(&set), 
		output(&out), 
		response(res),
		integral(0), 
		input_prev(0), 
		time_prev(0), cycle_time(100), 
		low(0), high(255), 
		on(OFF) {
			tune(p,i,d);
		}

// clamp to external limits
template <typename T>
void PID<T>::clamp(T& parameter) {
	if (parameter > high) parameter = high;
	else if (parameter < low) parameter = low;	
}

template <typename T>
bool PID<T>::compute() {
	if (!on) return false;
	unsigned long now = millis();
	// after 110 days, millis() will reset to 0
	unsigned long elapsed = now - time_prev;

	// in the middle of a cycle, ignore
	if (elapsed < cycle_time) return false;
	// proportional
	int error = (int)(*setpoint) - (int)(*input);

	// integral
	integral += ki * error;
	clamp(integral);

	// derivative on measurement to fix derivative kick
	T input_change = (*input - input_prev);

	T out = kp*error + integral - kd*input_change;
	clamp(out);
	*output = out;

	// update state
	input_prev = *input;
	time_prev = now;
	return true;
}

template <typename T>
void PID<T>::tune(double p, double i, double d) {
	double cycle_time_sec = ((double)cycle_time) / 1000;
	kp = p;
	ki = i * cycle_time_sec;
	kd = d / cycle_time_sec;
	if (response == NEGATIVE) {
		kp = -kp;
		ki = -ki;
		kd = -kd;
	} 
}

template <typename T>
void PID<T>::set_cycle(unsigned int new_cycle_time) {
	double adjust_ratio = ((double)new_cycle_time) / ((double)cycle_time);
	ki *= adjust_ratio;
	kd /= adjust_ratio;
	cycle_time = new_cycle_time;
}

template <typename T>
void PID<T>::set_limits(T l, T h) {
	// ignore troll value
	if (l >= h) return;
	low = l;
	high = h;
	// adjust exiting values
	if (on) {
		clamp(integral);
		clamp(*output);
	}
}

template <typename T>
void PID<T>::set_response(bool r) {
	if (on && r != response) {
		kp = -kp;
		ki = -ki;
		kd = -kd;
	}
	response = r;
}

template <typename T>
void PID<T>::start() {
	if (!on) {
		on = ON;
		reinitialize();
	}
}
template <typename T>
void PID<T>::stop() {on = OFF;}

template <typename T>
void PID<T>::reinitialize() {
	input_prev = *input;
	integral = *output;
	clamp(integral);
}

template <typename T>
bool PID<T>::get_onoff() const {return on;}

template <typename T>
bool PID<T>::get_response() const {return response;}

template class PID<int>;