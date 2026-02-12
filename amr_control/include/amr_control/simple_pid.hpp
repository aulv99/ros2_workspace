#ifndef SIMPLE_PID_HPP
#define SIMPLE_PID_HPP

#include <algorithm>
#include <cmath>

class SimplePID {
public:
    SimplePID(double kp, double ki, double kd, double min_val, double max_val)
    : kp_(kp), ki_(ki), kd_(kd), min_val_(min_val), max_val_(max_val) {
        prev_error_ = 0.0;
        integral_ = 0.0;
        first_run_ = true;
    }

    void set_coefficients(double kp, double ki, double kd) {
        kp_ = kp;
        ki_ = ki;
        kd_ = kd;
    }

    double calculate(double setpoint, double measured_value, double dt) {
        double error = setpoint - measured_value;

        // proportional
        double P = kp_ * error;

        // integral
        integral_ += error * dt;
        double I = ki_ * integral_;

        // derivative
        double D = 0.0;
        if (!first_run_ && dt > 0.0) {
            D = kd_ * (error - prev_error_) / dt;
        }

        // total output
        double output = P + I + D;

        // clamp output with limits
        output = std::clamp(output, min_val_, max_val_);

        // update state
        prev_error_ = error;
        first_run_ = false;

        return output;
    }

    void reset () {
        prev_error_ = 0.0;
        integral_ = 0.0;
        first_run_ = true;
    }



private:
    double kp_, ki_, kd_;
    double min_val_, max_val_;
    double prev_error_;
    double integral_;
    bool first_run_;
};

#endif // SIMPLE_PID_HPP