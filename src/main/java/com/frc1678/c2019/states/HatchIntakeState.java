package com.frc1678.c2019.states;

public class HatchIntakeState {
    public boolean backplateSolenoid = false;
    public boolean arrowheadSolenoid = false;

    public boolean leftHatchProxy = false;
    public boolean rightHatchProxy = false;

    public void setBackplate(boolean solenoid) {
        backplateSolenoid = solenoid;
    }

    public void setArrowhead(boolean solenoid) {
        arrowheadSolenoid = false;
    }

    public boolean hasHatch() {
        return leftHatchProxy && rightHatchProxy;
    }
}
