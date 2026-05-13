# Simplified Control Logic

This note points to the real ArduPilot implementations and rewrites the core idea as one simple function per topic.

## 1. PID controller

Real code:
- `libraries/AC_PID/AC_PID.cpp:196` - `AC_PID::update_all(...)`
- `libraries/AC_PID/AC_PID.cpp:368` - `AC_PID::get_ff()`

Important detail from the real implementation:
- `update_all()` returns only `P + I + D`
- `get_ff()` returns `FF + DFF`
- final controller output is often:

```cpp
final_output = update_all(...) + get_ff();
```

One-function simplified version:

```cpp
float pid_step(
    float target,
    float measurement,
    float dt,
    float kp,
    float ki,
    float kd,
    float kff,
    float kdff,
    float &integrator,
    float &prev_error,
    float &prev_target,
    float imax)
{
    float error = target - measurement;

    integrator += error * ki * dt;
    if (integrator > imax) integrator = imax;
    if (integrator < -imax) integrator = -imax;

    float derivative = (dt > 0.0f) ? (error - prev_error) / dt : 0.0f;
    float target_derivative = (dt > 0.0f) ? (target - prev_target) / dt : 0.0f;

    float p = kp * error;
    float i = integrator;
    float d = kd * derivative;
    float ff = kff * target;
    float dff = kdff * target_derivative;

    prev_error = error;
    prev_target = target;

    return p + i + d + ff + dff;
}
```

What comes out of this function:

```cpp
output = p + i + d + ff + dff;
```

## 2. Motor mixer

Real code:
- `libraries/AP_Motors/AP_MotorsMatrix.cpp:217` - `AP_MotorsMatrix::output_armed_stabilizing()`
- `libraries/AP_Motors/AP_MotorsMatrix.cpp:143` - `AP_MotorsMatrix::output_to_motors()`

Important detail from the real implementation:
- first, mixer computes per-motor thrust values into `_thrust_rpyt_out[i]`
- after that, those values are converted to actuator and PWM output

One-function simplified generic mixer:

```cpp
void mix_motors(
    float roll,
    float pitch,
    float yaw,
    float throttle,
    const float roll_factor[],
    const float pitch_factor[],
    const float yaw_factor[],
    const float throttle_factor[],
    float motor_out[],
    int motor_count)
{
    for (int i = 0; i < motor_count; i++) {
        float motor =
            throttle * throttle_factor[i] +
            roll     * roll_factor[i] +
            pitch    * pitch_factor[i] +
            yaw      * yaw_factor[i];

        if (motor < 0.0f) motor = 0.0f;
        if (motor > 1.0f) motor = 1.0f;

        motor_out[i] = motor;
    }
}
```

What comes out of this function:

```cpp
motor_out[i] = throttle * throttle_factor[i]
             + roll     * roll_factor[i]
             + pitch    * pitch_factor[i]
             + yaw      * yaw_factor[i];
```

So the result is not one scalar. The result is an array of motor commands:

```cpp
{ motor_out[0], motor_out[1], ..., motor_out[N-1] }
```

## 3. Closest match to the real ArduPilot flow

If we write both ideas in the same style as the real code, the final forms are:

```cpp
float final_pid_output = pid + feedforward;
```

```cpp
motor_out[i] = base_throttle + roll_mix + pitch_mix + yaw_mix;
```

In the real code these are expanded with filters, limits, slew limiting, yaw headroom, motor loss handling, thrust linearization, and PWM conversion.
