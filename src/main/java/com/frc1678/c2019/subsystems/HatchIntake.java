package com.frc1678.c2019.subsystems;

import com.frc1678.c2019.Constants;
import com.frc1678.c2019.loops.ILooper;
import com.frc1678.c2019.loops.Loop;
import com.frc1678.c2019.statemachines.HatchIntakeStateMachine;
import com.frc1678.c2019.states.HatchIntakeState;
import edu.wpi.first.wpilibj.Solenoid;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class HatchIntake extends Subsystem {
    private static HatchIntake mInstance;

    private final Solenoid mArrowheadSolenoid, mBackplateSolenoid;
    private final CarriageCanifier mCanifier = CarriageCanifier.getInstance();
    private HatchIntakeStateMachine.WantedAction mWantedAction = HatchIntakeStateMachine.WantedAction.NONE;
    private HatchIntakeState mCurrentState = new HatchIntakeState();
    private HatchIntakeStateMachine mStateMachine = new HatchIntakeStateMachine();

    private HatchIntake() {
        mArrowheadSolenoid = Constants.makeSolenoidForId(Constants.kArrowheadSolenoidId);
        mBackplateSolenoid = Constants.makeSolenoidForId(Constants.kBackplateSolenoidId);
    }

    public synchronized static HatchIntake getInstance() {
        if (mInstance == null) {
            mInstance = new HatchIntake();
        }

        return mInstance;
    }

    @Override
    public void outputTelemetry() {
        SmartDashboard.putBoolean("LeftHatchProxy", getLeftHatchProxy());
        SmartDashboard.putBoolean("RightHatchProxy", getRightHatchProxy());
    }

    @Override
    public void stop() {
    }

    @Override
    public void zeroSensors() {
    }

    private HatchIntakeState getCurrentState() {
        mCurrentState.leftHatchProxy = getLeftHatchProxy();
        mCurrentState.rightHatchProxy = getRightHatchProxy();
        return mCurrentState;
    }

    @Override
    public void registerEnabledLoops(ILooper enabledLooper) {
        Loop loop = new Loop() {
            @Override
            public void onStart(double timestamp) {
               mWantedAction = HatchIntakeStateMachine.WantedAction.HOLD;
            }

            @Override
            public void onLoop(double timestamp) {
                synchronized (HatchIntake.this) {
                    HatchIntakeState newState = mStateMachine.update(Timer.getFPGATimestamp(), mWantedAction, getCurrentState());
                    updateActuatorFromState(newState);
                }
            }

            @Override
            public void onStop(double timestamp) {
                mWantedAction = HatchIntakeStateMachine.WantedAction.HOLD;
            }
        };

        enabledLooper.register(loop);
    }

    private synchronized void updateActuatorFromState(HatchIntakeState state) {
        mArrowheadSolenoid.set(state.arrowheadSolenoid);
        mBackplateSolenoid.set(state.backplateSolenoid);
    }

    public synchronized boolean hasHatch() {
        return mStateMachine.debouncedHatch();
    }

    public synchronized boolean getIntakeOut() {
        return mBackplateSolenoid.get();
    }

    public boolean getLeftHatchProxy() {
        return mCanifier.getLeftHatchProxy();
    }

    public boolean getRightHatchProxy() {
        return mCanifier.getRightHatchProxy();
    }

    public synchronized void setState(HatchIntakeStateMachine.WantedAction action) {
        mWantedAction = action;
    }

    public HatchIntakeStateMachine.WantedAction getWantedAction() {
        return mWantedAction;
    }

    @Override 
    public void readPeriodicInputs() {
    }

    @Override 
    public void writePeriodicOutputs() {
    }

    @Override
    public boolean checkSystem() {
        return true;
    }
}
