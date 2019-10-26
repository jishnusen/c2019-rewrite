package com.frc1678.c2019.lib.control;

/**
 * PID controller that controls a control loop
 */

public class PIDController {

    private double kP;
    private double kI;
    private double kD;
    private double setpoint;
 
    private double prevError;
    private double integral;

    private double prevTime;
    
    public PIDController(double p, double i, double d, double sp) {
        setpoint = sp;
        kP = p;
        kI = i;
        kD = d;
    }

    public void reset() {
        setpoint = 0;
        prevError = 0;
        prevTime = 0;
        integral = 0;

    }

    public double update(double timestamp, double sensor) {
        prevTime = timestamp;
        prevError = setpoint - sensor;

        double dt = timestamp - prevTime;

        double error = setpoint - sensor;

        return (kP * error) + (kI * calculateIntegral(dt, error)) + (kD * calculateDerivative(dt, error));


    }

    // calculate functions could have been in the update lol

    private double calculateDerivative(double dt, double error) {
        double derivative = (error - prevError) / dt;
        prevError = error;
        return derivative;
    }

    private double calculateIntegral(double dt, double error) {
        integral += error * dt;
      //  prevError = error;
        return integral;
    }
}