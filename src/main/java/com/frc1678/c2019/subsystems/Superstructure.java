package com.frc1678.c2019.subsystems;

import com.frc1678.c2019.Constants;
import com.frc1678.c2019.Robot;
import com.frc1678.c2019.loops.ILooper;
import com.frc1678.c2019.loops.Loop;
import com.frc1678.c2019.statemachines.SuperstructureStateMachine;
import com.frc1678.c2019.states.HatchIntakeState;
import com.frc1678.c2019.states.SuperstructureCommand;
import com.frc1678.c2019.states.SuperstructureConstants;
import com.frc1678.c2019.states.SuperstructureState;
import com.team254.lib.util.LatchedBoolean;
import com.team254.lib.util.Util;
import edu.wpi.first.wpilibj.Solenoid;

/**
 * The superstructure subsystem is the overarching superclass containing all components of the superstructure: the
 * intake, hopper, feeder, shooter and LEDs. The superstructure subsystem also contains some miscellaneous hardware that
 * is located in the superstructure but isn't part of any other subsystems like the compressor, pressure sensor, and
 * hopper wall pistons.
 * <p>
 * Instead of interacting with subsystems like the feeder and intake directly, the {@link Robot} class interacts with
 * the superstructure, which passes on the commands to the correct subsystem.
 * <p>
 * The superstructure also coordinates actions between different subsystems like the feeder and shooter.
 *
 * @see Subsystem
 */
public class Superstructure extends Subsystem {
    static Superstructure mInstance = null;
    private SuperstructureState mState = new SuperstructureState();
    private Elevator mElevator = Elevator.getInstance();
    private Wrist mWrist = Wrist.getInstance();
    private HatchIntake mHatchIntake = HatchIntake.getInstance();
    private CargoIntake mCargoIntake = CargoIntake.getInstance();
    private Climber mClimber = Climber.getInstance();
    private SuperstructureStateMachine mStateMachine = new SuperstructureStateMachine();
    private SuperstructureStateMachine.WantedAction mWantedAction =
            SuperstructureStateMachine.WantedAction.IDLE;

    private boolean isWristJogging = false;
    private boolean isElevatorJogging = false;

    public synchronized static Superstructure getInstance() {
        if (mInstance == null) {
            mInstance = new Superstructure();
        }
        return mInstance;
    }

    @Override
    public boolean checkSystem() {
        return false;
    }

    @Override
    public void outputTelemetry() {

    }

    @Override
    public void stop() {

    }

    @Override
    public void zeroSensors() {

    }

    public synchronized SuperstructureStateMachine.SystemState getSuperStructureState() {
        return mStateMachine.getSystemState();
    }

    public synchronized SuperstructureState getObservedState() {
        return mState;
    }

    private synchronized void updateObservedState(SuperstructureState state) {
        state.height = mElevator.getInchesOffGround();
        state.angle = mWrist.getAngle();
        state.cargoIntakeOut = mCargoIntake.getIntakeOut();
        state.hatchIntakeOut = mHatchIntake.getIntakeOut();

        state.elevatorSentLastTrajectory = mElevator.hasFinishedTrajectory();
        state.wristSentLastTrajectory = mWrist.hasFinishedTrajectory();
    }

    // Update subsystems from planner
    synchronized void setFromCommandState(SuperstructureCommand commandState) {
        if (commandState.openLoopElevator) {
            mElevator.setOpenLoop(commandState.openLoopElevatorPercent);
        } else {
            if(isElevatorJogging) {
                mElevator.setPositionPID(commandState.height);
            } else {
                mElevator.setMotionMagicPosition(commandState.height);
            }
        }
        if (commandState.elevatorLowGear) {
            mElevator.setHangMode(true);
        } else {
            mElevator.setHangMode(false);
        }
        if(isWristJogging) {
            mWrist.setSetpointPositionPID(commandState.wristAngle, 0.0);
        } else {
            mWrist.setSetpointMotionMagic(commandState.wristAngle);
        }
    }

    @Override
    public void registerEnabledLoops(ILooper enabledLooper) {
        enabledLooper.register(new Loop() {
            private SuperstructureCommand mCommand;

            @Override
            public void onStart(double timestamp) {
                mStateMachine.resetManual();
            }

            @Override
            public void onLoop(double timestamp) {
                synchronized (Superstructure.this) {
                    updateObservedState(mState);

                    mStateMachine.setMaxHeight(SuperstructureConstants.kElevatorMaxHeight);
                    mCommand = mStateMachine.update(timestamp, mWantedAction, mState);
                    setFromCommandState(mCommand);
                }
            }

            @Override
            public void onStop(double timestamp) {

            }
        });
    }

    public synchronized double getScoringAngle() {
        return mStateMachine.getScoringAngle();
    }

    public synchronized double getScoringHeight() {
        return mStateMachine.getScoringHeight();
    }

    public synchronized void setDesiredHeight(double height) {
        isElevatorJogging = false;
        mStateMachine.setScoringHeight(height);
        mWantedAction = SuperstructureStateMachine.WantedAction.GO_TO_POSITION;
    }

    public synchronized void setDesiredAngle(double angle) {
        isWristJogging = false;
        mStateMachine.setScoringAngle(angle);
        mWantedAction = SuperstructureStateMachine.WantedAction.GO_TO_POSITION;
    }

    public synchronized void setElevatorJog(double relative_inches) {
        isElevatorJogging = true;
        mStateMachine.jogElevator(relative_inches);
        mWantedAction = SuperstructureStateMachine.WantedAction.GO_TO_POSITION;
    }

    public synchronized void setWristJog(double relative_degrees) {
        isWristJogging = true;
        mStateMachine.jogWrist(relative_degrees);
        mWantedAction = SuperstructureStateMachine.WantedAction.GO_TO_POSITION;
    }

    public synchronized void setElevatorLowGear() {
        mStateMachine.setManualWantsLowGear(true);
        mWantedAction = SuperstructureStateMachine.WantedAction.WANT_MANUAL;
    }

    public synchronized void setElevatorHighGear() {
        mStateMachine.setManualWantsLowGear(false);
        mWantedAction = SuperstructureStateMachine.WantedAction.WANT_MANUAL;
    }

    public synchronized void setHangThrottle(double throttle) {
        mStateMachine.setOpenLoopPower(throttle);
        mWantedAction = SuperstructureStateMachine.WantedAction.WANT_MANUAL;
    }

    public synchronized void setWantedAction(SuperstructureStateMachine.WantedAction wantedAction) {
        mWantedAction = wantedAction;
    }
}
