package com.frc1678.c2019.planners;

import com.frc1678.c2019.states.SuperstructureState;
import com.frc1678.c2019.states.SuperstructureConstants;
import com.team254.util.test.ControlledActuatorLinearSim;
import org.junit.jupiter.api.Test;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertTrue;


public class SuperstructureMotionPlannerTest {
    static final double ELEVATOR_MAX_HEIGHT = SuperstructureConstants.kElevatorMaxHeight;
    static final double ELEVATOR_MIN_HEIGHT = SuperstructureConstants.kElevatorMinHeight;
    static final double PIVOT_MAX_ANGLE = SuperstructureConstants.kWristMaxAngle;
    static final double PIVOT_MIN_ANGLE = SuperstructureConstants.kWristMinAngle;
    static final double ELEVATOR_VELOCITY = 70.0;
    static final double PIVOTY_VELOCITY = 180.0;
    static final double DELTA_T = 0.005;
    private static final double EPSILON = 1e-8;

    SuperstructureMotionPlanner planner = new SuperstructureMotionPlanner();
    SuperstructureState desiredState = new SuperstructureState();
    SuperstructureState simulatedState = new SuperstructureState();
    ControlledActuatorLinearSim elevatorSim = new ControlledActuatorLinearSim(ELEVATOR_MIN_HEIGHT,
            ELEVATOR_MAX_HEIGHT, ELEVATOR_VELOCITY);
    ControlledActuatorLinearSim pivotSim = new ControlledActuatorLinearSim(PIVOT_MIN_ANGLE, PIVOT_MAX_ANGLE,
            PIVOTY_VELOCITY);

    @Test
    public void testPlannerBootsToHome() {
        planner.setDesiredState(desiredState, simulatedState);
        SuperstructureState commandedState = planner.update(new SuperstructureState());
        assertEquals(ELEVATOR_MIN_HEIGHT, commandedState.height, EPSILON, "height must not go lower than low limit");
    }

    @Test
    public void testElevatorBottomLimit() {
        desiredState.height = -10;
        desiredState.angle = 0;
        simulatedState.height = ELEVATOR_MIN_HEIGHT;
        simulatedState.angle = 0;
        planner.reset(simulatedState);
        planner.setDesiredState(desiredState, simulatedState);
        SuperstructureState commandedState = planner.update(simulatedState);
        assertEquals(PIVOT_MIN_ANGLE, commandedState.angle, EPSILON, "angle must be homed");
        assertEquals(ELEVATOR_MIN_HEIGHT, commandedState.height, EPSILON, "height must be homed");
    }

    @Test
    public void testCanMovePivotAtZeroHeight() {
        // 0 to 180
        desiredState.height = ELEVATOR_MIN_HEIGHT;
        desiredState.angle = 180;
        simulatedState.height = ELEVATOR_MIN_HEIGHT;
        simulatedState.angle = 0;
        planner.setDesiredState(desiredState, simulatedState);
        SuperstructureState commandedState = planner.update(simulatedState);
        assertEquals(180, commandedState.angle, EPSILON, "angle must be correct");
        assertEquals(ELEVATOR_MIN_HEIGHT, commandedState.height, EPSILON, "height must be correct");

        // 180 to 0
        desiredState.height = ELEVATOR_MIN_HEIGHT;
        desiredState.angle = 0;
        simulatedState.height = ELEVATOR_MIN_HEIGHT;
        simulatedState.angle = 180;
        planner.setDesiredState(desiredState, simulatedState);
        commandedState = planner.update(simulatedState);
        assertEquals(0, commandedState.angle, EPSILON, "angle must be correct");
        assertEquals(ELEVATOR_MIN_HEIGHT, commandedState.height, EPSILON, "height must be correct");

        // 0 to 90
        desiredState.height = ELEVATOR_MIN_HEIGHT;
        desiredState.angle = 90;
        simulatedState.height = ELEVATOR_MIN_HEIGHT;
        simulatedState.angle = 0;
        planner.setDesiredState(desiredState, simulatedState);
        commandedState = planner.update(simulatedState);
        assertEquals(90, commandedState.angle, EPSILON, "angle must be correct");
        assertEquals(ELEVATOR_MIN_HEIGHT, commandedState.height, EPSILON, "height must be correct");
    }

    @Test
    public void testPivotSims() {
        SuperstructureState currentState = new SuperstructureState();
        desiredState.height = ELEVATOR_MIN_HEIGHT;
        desiredState.angle = 180;
        simulatedState.height = ELEVATOR_MIN_HEIGHT;
        simulatedState.angle = 0;
        pivotSim.reset(simulatedState.angle);
        planner.reset(currentState);
        planner.setDesiredState(desiredState, simulatedState);

        for (double ts = 0; ts < 3.0; ts += DELTA_T) {
            SuperstructureState command = planner.update(currentState);
            pivotSim.setCommandedPosition(command.angle);
            double pivotAngle = pivotSim.update(DELTA_T);
            double raw = (ts + DELTA_T) * PIVOT_MAX_ANGLE;
            double expected = raw > 180 ? 180 : raw;
            assertEquals(expected, pivotAngle, EPSILON, "angle must be correct");
            currentState.angle = pivotAngle;
        }
    }

    @Test
    public void testWristCapsForPassthrough() {
        // Start at the top
        simulatedState = new SuperstructureState();
        simulatedState.angle = PIVOT_MIN_ANGLE;
        simulatedState.height = ELEVATOR_MAX_HEIGHT;
        pivotSim.reset(simulatedState.angle);
        elevatorSim.reset(simulatedState.height);

        // desire to pass through at ground height
        desiredState.height = ELEVATOR_MIN_HEIGHT;
        desiredState.angle = PIVOT_MAX_ANGLE;

        planner.setDesiredState(desiredState, simulatedState);

        // Test that wrist is never inside the stage unless elevator at ground
        for (double ts = 0; ts < 5; ts += DELTA_T) {
            SuperstructureState command = planner.update(simulatedState);
            pivotSim.setCommandedPosition(command.angle);
            simulatedState.angle = pivotSim.update(DELTA_T);
            elevatorSim.setCommandedPosition(command.height);
            simulatedState.height = elevatorSim.update(DELTA_T);

            boolean elevatorUnsafe = (simulatedState.height > SuperstructureConstants.kElevatorSafeHeight + 1);
            boolean wristSafe = !simulatedState.wristPassingThrough() && !command.wristPassingThrough();
            if (elevatorUnsafe) {
                if (!wristSafe) {
                    System.out.println("Wrist illegal: " + simulatedState.angle + " " + command.angle + " " + simulatedState.height);
                }
                assertTrue(wristSafe, "wrist must be safe while moving");
            }
        }

        assertEquals(desiredState.height, simulatedState.height);
        assertEquals(desiredState.angle, simulatedState.angle);
    }

    @Test
    public void testWristCapsForPassthroughReverse() {
        // Start passed through at min height
        simulatedState = new SuperstructureState();
        simulatedState.angle = PIVOT_MAX_ANGLE;
        simulatedState.height = ELEVATOR_MIN_HEIGHT;
        pivotSim.reset(simulatedState.angle);
        elevatorSim.reset(simulatedState.height);

        // desire to move to top, wrist out straight
        desiredState.height = ELEVATOR_MAX_HEIGHT;
        desiredState.angle = PIVOT_MIN_ANGLE;

        planner.setDesiredState(desiredState, simulatedState);

        // Test that wrist never passes through unless elevator at ground
        for (double ts = 0; ts < 5; ts += DELTA_T) {
            SuperstructureState command = planner.update(simulatedState);
            pivotSim.setCommandedPosition(command.angle);
            simulatedState.angle = pivotSim.update(DELTA_T);
            elevatorSim.setCommandedPosition(command.height);
            simulatedState.height = elevatorSim.update(DELTA_T);

            boolean elevatorUnsafe = (simulatedState.height > SuperstructureConstants.kElevatorSafeHeight + 1);
            boolean wristSafe = !simulatedState.wristPassingThrough() && !command.wristPassingThrough();
            if (elevatorUnsafe) {
                if (!wristSafe) {
                    System.out.println("Wrist illegal: " + simulatedState.angle + " " + command.angle + " " + simulatedState.height);
                }
                assertTrue(wristSafe, "wrist must be safe while moving");
            }
        }
        
        assertEquals(desiredState.height, simulatedState.height);
        assertEquals(desiredState.angle, simulatedState.angle);

        desiredState.height = ELEVATOR_MAX_HEIGHT;
        desiredState.angle = PIVOT_MAX_ANGLE;

        planner.setDesiredState(desiredState, simulatedState);

        // Now we want to end up at the same height, but passed through. The elevator should go down to pass through and then spring up
        boolean sprung_down = false;
        for (double ts = 0; ts < 20; ts += DELTA_T) {
            SuperstructureState command = planner.update(simulatedState);
            pivotSim.setCommandedPosition(command.angle);
            simulatedState.angle = pivotSim.update(DELTA_T);
            elevatorSim.setCommandedPosition(command.height);
            simulatedState.height = elevatorSim.update(DELTA_T);

            boolean elevatorUnsafe = (simulatedState.height > SuperstructureConstants.kElevatorSafeHeight + 1);
            boolean wristSafe = !simulatedState.wristPassingThrough() && !command.wristPassingThrough();
            if (elevatorUnsafe) {
                if (!wristSafe) {
                    System.out.println("Wrist illegal: " + simulatedState.angle + " " + command.angle + " " + simulatedState.height);
                }
                assertTrue(wristSafe, "wrist must be safe while moving");
            } else {
                sprung_down = true; // Elevator came down to be safe for the passthrough (should be a redundant check)
            }
        }

        assertEquals(desiredState.height, simulatedState.height);
        assertEquals(desiredState.angle, simulatedState.angle);
        assertTrue(sprung_down);
    }
}