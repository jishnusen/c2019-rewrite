package com.frc1678.c2019.planners;

import com.frc1678.c2019.states.SuperstructureState;
import com.frc1678.c2019.states.SuperstructureConstants;
import com.team254.lib.util.Util;

import java.util.LinkedList;
import java.util.Optional;

public class SuperstructureMotionPlanner {
    class SubCommand {
        public SubCommand(SuperstructureState endState) {
            mEndState = endState;
        }

        public SuperstructureState mEndState;
        public double mHeightThreshold = 1.0;
        public double mWristThreshold = 5.0;

        public boolean isFinished(SuperstructureState currentState) {
            return mEndState.isInRange(currentState, mHeightThreshold, mWristThreshold);
        }
    }

    class WaitForElevatorApproachingSubcommand extends SubCommand {
        public WaitForElevatorApproachingSubcommand(SuperstructureState endState) {
            super(endState);
            mHeightThreshold = SuperstructureConstants.kElevatorApproachingThreshold;
        }

        @Override
        public boolean isFinished(SuperstructureState currentState) {
            return mEndState.isInRange(currentState, mHeightThreshold, Double.POSITIVE_INFINITY);
        }
    }

    class WaitForPassThroughSafe extends SubCommand {
        boolean forwards;
        public WaitForPassThroughSafe(SuperstructureState endState, SuperstructureState currentState) {
            super(endState);
            forwards = endState.angle > SuperstructureConstants.kWristSafeBackwardsAngle && currentState.angle < SuperstructureConstants.kWristSafeForwardsAngle;
        }

        @Override
        public boolean isFinished(SuperstructureState currentState) {
            if (forwards) {
                return currentState.angle > SuperstructureConstants.kWristSafeBackwardsAngle;
            } else {
                return currentState.angle < SuperstructureConstants.kWristSafeForwardsAngle;
            }
        }
    }

    class WaitForFinalSetpointSubcommand extends SubCommand {
        public WaitForFinalSetpointSubcommand(SuperstructureState endState) {
            super(endState);
        }

        @Override
        public boolean isFinished(SuperstructureState currentState) {
            return currentState.elevatorSentLastTrajectory && currentState.wristSentLastTrajectory;
        }
    }

    protected SuperstructureState mCommandedState = new SuperstructureState();
    protected SuperstructureState mIntermediateCommandState = new SuperstructureState();
    protected LinkedList<SubCommand> mCommandQueue = new LinkedList<>();
    protected Optional<SubCommand> mCurrentCommand = Optional.empty();

    public synchronized boolean setDesiredState(SuperstructureState desiredStateIn, SuperstructureState currentState) {
        SuperstructureState desiredState = new SuperstructureState(desiredStateIn);

        // Limit illegal inputs.
        desiredState.angle = Util.limit(desiredState.angle, SuperstructureConstants.kWristMinAngle,
                SuperstructureConstants.kWristMaxAngle);
        desiredState.height = Util.limit(desiredState.height, SuperstructureConstants.kElevatorMinHeight,
                SuperstructureConstants.kElevatorMaxHeight);

        // Immediate return, totally illegal commands.
        if (desiredState.inIllegalIntakeZone()) {
            // Desired state is not legal.  Return false, let the caller deal with it.
            return false;
        }

        // Everything beyond this is probably do-able; clear queue
        mCommandQueue.clear();

        final boolean passingThru = (desiredState.angle > SuperstructureConstants.kWristSafeBackwardsAngle
                && currentState.angle < SuperstructureConstants.kWristSafeBackwardsAngle) ||
            (desiredState.angle < SuperstructureConstants.kWristSafeForwardsAngle &&
             currentState.angle > SuperstructureConstants.kWristSafeForwardsAngle);
        final double firstElevatorHeight = passingThru ? 0.0 : desiredState.height;
        double firstWristAngle = desiredState.angle;
        if (passingThru && currentState.height > SuperstructureConstants.kElevatorSafeHeight) {
            if (desiredState.angle > SuperstructureConstants.kWristSafeForwardsAngle) {
                firstWristAngle = Math.min(SuperstructureConstants.kWristSafeForwardsAngle, desiredState.angle);
            } else if (desiredState.angle < SuperstructureConstants.kWristSafeBackwardsAngle) {
                firstWristAngle = Math.max(desiredState.angle, SuperstructureConstants.kWristSafeBackwardsAngle);
            }
        }

        if (passingThru) {
            // Get down to pass through first
            mCommandQueue.add(new WaitForElevatorApproachingSubcommand(new SuperstructureState(firstElevatorHeight, firstWristAngle, false, false)));

            // Start passing through after that
            mCommandQueue.add(new WaitForPassThroughSafe(new SuperstructureState(firstElevatorHeight,
                    desiredState.angle, false, false), new SuperstructureState(currentState)));
        }

        // Go to the goal.
        mCommandQueue.add(new WaitForFinalSetpointSubcommand(desiredState));

        // Reset current command to start executing on next iteration
        mCurrentCommand = Optional.empty();

        return true; // this is a legal move
    }

    void reset(SuperstructureState currentState) {
        mIntermediateCommandState = currentState;
        mCommandQueue.clear();
        mCurrentCommand = Optional.empty();
    }

    public boolean isFinished(SuperstructureState currentState) {
        return mCurrentCommand.isPresent() && mCommandQueue.isEmpty() && currentState.wristSentLastTrajectory &&
                currentState.elevatorSentLastTrajectory;
    }

    public SuperstructureState update(SuperstructureState currentState) {
        if (!mCurrentCommand.isPresent() && !mCommandQueue.isEmpty()) {
            mCurrentCommand = Optional.of(mCommandQueue.remove());
        }

        if (mCurrentCommand.isPresent()) {
            SubCommand subCommand = mCurrentCommand.get();
            mIntermediateCommandState = subCommand.mEndState;
            if (subCommand.isFinished(currentState) && !mCommandQueue.isEmpty()) {
                // Let the current command persist until there is something in the queue. or not. desired outcome
                // unclear.
                mCurrentCommand = Optional.empty();
            }
        } else {
            mIntermediateCommandState = currentState;
        }

        mCommandedState.angle = Util.limit(mIntermediateCommandState.angle, SuperstructureConstants.kWristMinAngle,
                SuperstructureConstants.kWristMaxAngle);
        mCommandedState.height = Util.limit(mIntermediateCommandState.height, SuperstructureConstants
                .kElevatorMinHeight, SuperstructureConstants.kElevatorMaxHeight);

        return mCommandedState;
    }
}
