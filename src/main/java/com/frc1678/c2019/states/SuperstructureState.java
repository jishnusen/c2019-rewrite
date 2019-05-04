package com.frc1678.c2019.states;

import com.team254.lib.util.Util;

public class SuperstructureState {
    public double height = SuperstructureConstants.kElevatorMinHeight;
    public double angle = SuperstructureConstants.kWristMinAngle;
    
    public boolean hatchIntakeOut = false;
    public boolean cargoIntakeOut = false;

    public boolean hasHatch = false;
    public boolean hasCargo = false;
    public boolean elevatorSentLastTrajectory = false;
    public boolean wristSentLastTrajectory = false;

    public SuperstructureState(double height, double angle, boolean hatchIntakeOut, boolean cargoIntakeOut) {
        this.height = height;
        this.angle = angle;
        this.hatchIntakeOut = hatchIntakeOut;
        this.cargoIntakeOut = cargoIntakeOut;
    }

    public SuperstructureState(double height, double angle) {
        this(height, angle, false, false);
    }

    public SuperstructureState(SuperstructureState other) {
        this.height = other.height;
        this.angle = other.angle;
        this.hatchIntakeOut = other.hatchIntakeOut;
        this.cargoIntakeOut = other.cargoIntakeOut;
    }
    
    public SuperstructureState() {
        this(SuperstructureConstants.kElevatorMinHeight, SuperstructureConstants.kWristMinAngle, false, false);
    }

    public boolean wristPassingThrough() {
        return angle > SuperstructureConstants.kWristSafeForwardsAngle && angle < SuperstructureConstants.kWristSafeBackwardsAngle; 
    }

    public boolean inIllegalIntakeZone() {
        return (hatchIntakeOut || cargoIntakeOut) && wristPassingThrough();
    }

    public boolean isInRange(SuperstructureState otherState, double heightThreshold, double wristThreshold) {
        return Util.epsilonEquals(otherState.height, height, heightThreshold) &&
                Util.epsilonEquals(otherState.angle, angle, wristThreshold);

    }

    @Override
    public String toString() {
        return "" + height + " / " + angle + " / " + hatchIntakeOut + " / "+ cargoIntakeOut + " / " + hasHatch + " / " + hasCargo;
    }
}
