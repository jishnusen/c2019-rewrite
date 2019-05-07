package com.frc1678.c2019.statemachines;

import com.frc1678.c2019.statemachines.HatchIntakeStateMachine.WantedAction;
import com.frc1678.c2019.statemachines.HatchIntakeStateMachine.SystemState;

import com.frc1678.c2019.states.HatchIntakeState;
import org.junit.jupiter.api.Test;

import static org.junit.jupiter.api.Assertions.assertEquals;
import static org.junit.jupiter.api.Assertions.assertFalse;
import static org.junit.jupiter.api.Assertions.assertTrue;

class HatchIntakeStateMachineTest {
    HatchIntakeStateMachine sm = new HatchIntakeStateMachine();
    HatchIntakeStateMachine.WantedAction action = WantedAction.NONE;
    HatchIntakeState simState = new HatchIntakeState();

    private static final double EPSILON = 1e-8;

    @Test
    public void testBoot() {
        HatchIntakeState commandedState = sm.update(0, action, simState);
        assertTrue(commandedState.arrowheadSolenoid, "must hold preloaded hatch");
        assertFalse(commandedState.backplateSolenoid, "must not stick intake out");
    }

    @Test
    public void testSimpleIntake() {
        action = WantedAction.INTAKE;
        HatchIntakeState commandedState = sm.update(0, action, simState);
        commandedState = sm.update(0.1, action, simState);
        action = WantedAction.NONE;

        assertTrue(commandedState.arrowheadSolenoid);
        assertTrue(commandedState.backplateSolenoid);
        assertEquals(sm.getCurrentState(), SystemState.INTAKING);

        simState.leftHatchProxy = true;
        simState.rightHatchProxy = true;

        commandedState = sm.update(0.2, action, simState);
        action = WantedAction.NONE;
        commandedState = sm.update(0.3, action, simState);

        assertTrue(commandedState.arrowheadSolenoid);
        assertFalse(commandedState.backplateSolenoid);
        assertEquals(sm.getCurrentState(), SystemState.CARRYING);
    }

    @Test
    public void testGotHatchInPrepScore() {
        action = WantedAction.PREP_SCORE;
        simState.leftHatchProxy = false;
        simState.rightHatchProxy = false;

        HatchIntakeState commandedState = sm.update(0, action, simState);
        commandedState = sm.update(0, action, simState);
        action = WantedAction.NONE;

        assertTrue(commandedState.arrowheadSolenoid, "arrowhead out for hatch");
        assertTrue(commandedState.backplateSolenoid, "backplate out for hatch");

        simState.leftHatchProxy = true;
        simState.rightHatchProxy = true;

        assertTrue(simState.hasHatch(), "needs to think it has a hatch");
        
        sm.update(0, action, simState);
        commandedState = sm.update(0, action, simState);

        assertTrue(commandedState.arrowheadSolenoid, "must hold hatch");
        assertTrue(commandedState.backplateSolenoid, "shouldn't retract to hold hatch");
        assertEquals(sm.getCurrentState(), SystemState.PREPPING_SCORE, "no transition, we are now in prep score");
    }

    @Test
    public void testTimedOuttake() {
        action = WantedAction.NONE;
        simState.leftHatchProxy = true;
        simState.rightHatchProxy = true;

        HatchIntakeState commandedState = sm.update(0, action, simState);
        commandedState = sm.update(0, action, simState);

        assertTrue(commandedState.arrowheadSolenoid, "must hold hatch");
        assertFalse(commandedState.backplateSolenoid, "retract to hold hatch");
        assertEquals(sm.getCurrentState(), SystemState.CARRYING);

        action = WantedAction.PREP_SCORE;
        commandedState = sm.update(0, action, simState);
        action = WantedAction.NONE;
        commandedState = sm.update(0, action, simState);

        assertTrue(commandedState.arrowheadSolenoid, "must hold hatch");
        assertTrue(commandedState.backplateSolenoid, "stick out to score hatch");
        assertEquals(sm.getCurrentState(), SystemState.PREPPING_SCORE);

        action = WantedAction.SCORE;
        commandedState = sm.update(0, action, simState);
        action = WantedAction.NONE;
        commandedState = sm.update(0, action, simState);
        assertFalse(commandedState.arrowheadSolenoid, "immediately score");
        assertTrue(commandedState.backplateSolenoid, "stick out to score hatch");
        commandedState = sm.update(1.0, action, simState);
        assertFalse(commandedState.arrowheadSolenoid, "immediately score");
        assertFalse(commandedState.backplateSolenoid, "pullout");
    }
}