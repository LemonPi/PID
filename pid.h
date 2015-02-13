#pragma once
const bool OFF = 0;
const bool ON = 1;
const bool NEGATIVE = 0;
const bool POSITIVE = 1;

template <typename T = int>
class PID {
	// not going to change input and setpoint inside
	volatile const T* input;
	const T* setpoint;
	T* output;
	// adjustable constants
	double kp, ki, kd;
	
	T integral, input_prev;
	unsigned long time_prev;
	unsigned int cycle_time;	// 100ms 0.1s
	// limits
	T low, high;

	// byte flags;
	bool on;
	// what's the relation between input and output the controller should expect
	// POSITIVE means increasing output causes increasing input
	bool response;

	// clamp to external limits
	void clamp(T& parameter);

public:
	// by default off
	PID(volatile const T& in, const T& set, T& out, double p = 0.5, double i = 0, double d = 0, bool res = POSITIVE);

	bool compute();
	// change parameters (1/s)
	void tune(double p, double i, double d);
	// change cycle time
	void set_cycle(unsigned int new_cycle_time);
	// change output limits
	void set_limits(T l, T h);
	void set_response(bool r);

	void start();
	void stop();

	void reinitialize();

	bool get_onoff() const;
	bool get_response() const;
};