// PID automated tuning (Ziegler-Nichols/relay method) for Arduino and compatible boards
// Copyright (c) 2016-2020 jackw01
// This code is distrubuted under the MIT License, see LICENSE for details

#ifndef PIDAUTOTUNER_H_
#define PIDAUTOTUNER_H_


#ifndef M_PI
#define M_PI 3.14159265358979323846
#endif


typedef struct {
	float pid_autotune_kp;
	float pid_autotune_ki;
	float pid_autotune_kd;
} pid_autotune_results_t;


class PIDAutotuner {
public:
	// Constants for Ziegler-Nichols tuning mode
	typedef enum {
		ZNModeBasicPID,
		ZNModeLessOvershoot,
		ZNModeNoOvershoot
	} ZNMode;

	typedef struct {
		float pid_autotune_target;
		long pid_autotune_interval;
		float pid_autotune_outrange_min;
		float pid_autotune_outrange_max;
		ZNMode pid_autotune_zn;
		int pid_autotune_tune_cycles;
	} pid_autotune_params_t;

	PIDAutotuner();

	// Configure parameters for PID tuning
	// See README for more details - https://github.com/jackw01/arduino-pid-autotuner/blob/master/README.md
	// targetInputValue: the target value to tune to
	// loopInterval: PID loop interval in microseconds - must match whatever the PID loop being tuned runs at
	// outputRange: min and max values of the output that can be used to control the system (0, 255 for analogWrite)
	// znMode: Ziegler-Nichols tuning mode (znModeBasicPID, znModeLessOvershoot, znModeNoOvershoot)
	// tuningCycles: number of cycles that the tuning runs for (optional, default is 10)
	void setTargetInputValue(float target);
	void setLoopInterval(long interval);
	void setOutputRange(float min, float max);
	void setZNMode(ZNMode zn);
	void setTuningCycles(int tuneCycles);

	void setTuningParams(pid_autotune_params_t pidParams);

	// Must be called immediately before the tuning loop starts
	void startTuningLoop(unsigned long us);

	// Automatically tune PID
	// This function must be run in a loop at the same speed as the PID loop being tuned
	// See README for more details - https://github.com/jackw01/arduino-pid-autotuner/blob/master/README.md
	float tunePID(float input, unsigned long us);

	// Get resulting PID constants after most recent tuning
	float getKp();
	float getKi();
	float getKd();

	void getPIDParams(pid_autotune_results_t *pidResults);

	bool isFinished(); // Is the tuning finished?

	int getCycle(); // return tuning cycle

private:
	float targetInputValue = 0;
	float loopInterval = 0;
	float minOutput;
	float maxOutput;
	ZNMode znMode = ZNModeNoOvershoot;
	int cycles = 10;

	// See startTuningLoop()
	int i;
	bool output;
	float outputValue;
	long microseconds;
	long t1;
	long t2;
	long tHigh;
	long tLow;
	float max;
	float min;
	float pAverage;
	float iAverage;
	float dAverage;

	float kp;
	float ki;
	float kd;
};


#endif	/* !PIDAUTOTUNER_H_ */

