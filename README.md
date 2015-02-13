# PID
General PID control for volatile input (interrupt driven).

Based on Brett Beauregard's [PID_V1 library](https://github.com/br3ttb/Arduino-PID-Library/),
but with a more efficient compute (mainly by getting rid of his liberal use of doubles).

# Instructions
For arduino users, place directory inside My Documents/Arduino/libraries
```C++
#include <pid.h>

// volatile input that you want to listen to
volatile int tick_l = 0;

// target of 100 ticks per second
int target_l = 100;

// PWM output to adjust input level
int motor_level = 0;

// templated on input, setpoint, and output type
PID<int> motor_control_left(tick_l, target_l, motor_level);

// can be more specific and assign tuning parameters and output/input relations
PID<int> motor_control_alt(tick_l, target_l, motor_level, 2.0, 0, 0.5, NEGATIVE);
// negative relation means as output increases input should be expected to decrease


void setup() {

  // start monitoring input and give output feedback
  motor_control_left.start();
  
  // set cycle time to be 50ms = 20Hz (by default 100ms)
  motor_control_left.set_cycle(50);
  
  // tune with different parameters
  motor_control_left.tune(2.0, 0, 0.5);
}

void loop() {
  
  // simultaneously run PID and check if clock cycle occured
  if (motor_control_left.compute()) {
    Serial.println(tick_l);
    tick_l = 0;
  }
}
