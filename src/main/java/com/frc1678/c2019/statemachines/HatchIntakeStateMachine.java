package com.frc1678.c2019.statemachines;

import com.frc1678.c2019.states.HatchIntakeState;
import com.team254.lib.util.TimeDelayedBoolean;

public class HatchIntakeStateMachine {
    private final static double kLostHatchTime = 0.50;

    public enum WantedAction {
        NONE,
        INTAKE,
        HOLD,
        SCORE,
        PREP_SCORE,
        HANDOFF,
    }

    public enum SystemState {
        IDLE,
        INTAKING,
        CARRYING,
        OUTTAKING,
        PREPPING_SCORE,
        HANDOFF_INTAKING,
    }
    
    private SystemState mSystemState = SystemState.IDLE;
    private double mCurrentStateStartTime = 0;

    private TimeDelayedBoolean mLastSeenHatchLeft = new TimeDelayedBoolean();
    private TimeDelayedBoolean mLastSeenHatchRight = new TimeDelayedBoolean();
    private boolean mDebouncedHatch = false;

    public synchronized boolean debouncedHatch() {
        return mDebouncedHatch;
    }

    public HatchIntakeState update(double timestamp, WantedAction wantedAction, HatchIntakeState currentState) {
        synchronized (HatchIntakeStateMachine.this) {
            SystemState newState = mSystemState;
            double timeInState = timestamp - mCurrentStateStartTime;

            switch (mSystemState) {
                case IDLE:
                    currentState.backplateSolenoid = false;
                    currentState.arrowheadSolenoid = true;
                    if (currentState.hasHatch()) {
                        newState = SystemState.CARRYING;
                        currentState.backplateSolenoid = false;
                    }
                    break;
                case INTAKING:
                    currentState.backplateSolenoid = true;
                    currentState.arrowheadSolenoid = true;
                    if (currentState.hasHatch()) {
                        newState = SystemState.CARRYING;
                        currentState.backplateSolenoid = false;
                    }
                    break;
                case CARRYING:
                    currentState.backplateSolenoid = false;
                    currentState.arrowheadSolenoid = true;
                    break;
                case OUTTAKING:
                    currentState.backplateSolenoid = timeInState < 0.1;
                    currentState.arrowheadSolenoid = false;
                    if (timeInState > 0.8) {
                        newState = SystemState.IDLE;
                    }
                    break;
                case PREPPING_SCORE:
                    currentState.backplateSolenoid = true;
                    currentState.arrowheadSolenoid = true;
                    break;
                case HANDOFF_INTAKING:
                    currentState.backplateSolenoid = false;
                    currentState.arrowheadSolenoid = false;
                    if (currentState.hasHatch()) {
                        newState = SystemState.CARRYING;
                        currentState.backplateSolenoid = false;
                    }
                    break;
                default:
                    System.out.println("Unexpected intake system state: " + mSystemState);
            }

            if (newState != mSystemState) {
                System.out.println(timestamp + ": Intake changed state: " + mSystemState + " -> " + newState);
                mSystemState = newState;
                mCurrentStateStartTime = timestamp;
            }
            boolean left_debounced = mLastSeenHatchLeft.update(currentState.leftHatchProxy, kLostHatchTime);
            boolean right_debounced = mLastSeenHatchRight.update(currentState.rightHatchProxy, kLostHatchTime);

            mDebouncedHatch = left_debounced && right_debounced;

            handleTransitions(wantedAction);
        }

        return currentState;
    }

    public SystemState getCurrentState() {
        return mSystemState;
    }

    private void handleTransitions(WantedAction wantedAction) {
        switch (wantedAction) {
            case NONE:
                break;
            case INTAKE:
                mSystemState = SystemState.INTAKING;
                break;
            case HOLD:
                mSystemState = SystemState.CARRYING;
                break;
            case SCORE:
                mSystemState = SystemState.OUTTAKING;
                break;
            case PREP_SCORE:
                mSystemState = SystemState.PREPPING_SCORE;
                break;
            case HANDOFF:
                mSystemState = SystemState.HANDOFF_INTAKING;
                break;
        }
    }
}

