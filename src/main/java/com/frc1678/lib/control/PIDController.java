package com.frc1678.lib.control;

import com.team254.lib.util.Util;

public class PIDController {
    public static class PIDGains {
        public double p;
        public double i;
        public double d;

        public PIDGains(double p, double i, double d) {
            this.p = p;
            this.i = i;
            this.d = d;
        }
    }

    private PIDGains gains;

    private double setpoint;

    private boolean continuous = true;
    private double inputRange = Double.POSITIVE_INFINITY;
    private double minOutput = Double.NEGATIVE_INFINITY;
    private double maxOutput = Double.POSITIVE_INFINITY;

    private double lastError = Double.NaN;
    private double integralAccum = 0.0;
    private double integralRange = Double.POSITIVE_INFINITY;

    public PIDController(PIDGains constants) {
        gains = constants;
    }

    public double calculate(double current, double dt) {
        double error = setpoint - current;
        if (continuous) {
            error %= inputRange;
            if (Math.abs(error) > inputRange / 2.0) {
                if (error > 0.0) {
                    error -= inputRange;
                } else {
                    error += inputRange;
                }
            }
        }

        double integral = 0.0;
        if (Math.abs(error) > integralRange / 2.0) {
            integral = integralAccum + error * dt;
        }
        integralAccum = integral;

        double derivative = 0.0;
        if (Double.isFinite(lastError)) {
            derivative = (error - lastError) / dt;
        }
        lastError = error;

        return Util.limit(gains.p * error + gains.i * integral + gains.d * derivative,
                minOutput, maxOutput);
    }

    public void reset() {
        lastError = Double.NaN;
        integralAccum = 0.0;
    }

    public double getSetpoint() {
        return setpoint;
    }

    public void setSetpoint(double setpoint) {
        this.setpoint = setpoint;
    }

    public void setContinuous(boolean continuous) {
        this.continuous = continuous;
    }

    public void setInputRange(double minInput, double maxInput) {
        this.inputRange = maxInput - minInput;
    }

    public void setIntegralRange(double integralRange) {
        this.integralRange = integralRange;
    }

    /**
     * Sets the output range for the controller. Outputs will be clamped between these two values.
     *
     * @param min the minimum allowable output value
     * @param max the maximum allowable output value
     */
    public void setOutputRange(double min, double max) {
        if (max < min) {
            throw new IllegalArgumentException("Minimum output cannot be greater than maximum output");
        }

        minOutput = min;
        maxOutput = max;
    }
}