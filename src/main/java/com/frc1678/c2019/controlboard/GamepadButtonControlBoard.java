package com.frc1678.c2019.controlboard;

import com.frc1678.c2019.Constants;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.Hand;
import edu.wpi.first.wpilibj.GenericHID.RumbleType;

public class GamepadButtonControlBoard implements IButtonControlBoard {
    private static GamepadButtonControlBoard mInstance = null;

    public static GamepadButtonControlBoard getInstance() {
        if (mInstance == null) {
            mInstance = new GamepadButtonControlBoard();
        }

        return mInstance;
    }

    private XboxController mJoystick;

    private GamepadButtonControlBoard() {
        mJoystick = new XboxController(Constants.kButtonGamepadPort);
    }

    // Elevator-Wrist combos
    @Override
    public boolean goToGround() {
        return mJoystick.getPOV() == 180; // South
    }

    @Override
    public boolean goToStow() {
        return mJoystick.getPOV() == 0;
    }

    @Override
    public boolean goToFirstLevel() {
        return mJoystick.getAButton();
    }

    @Override
    public boolean goToSecondLevel() {
        return mJoystick.getBButton();
    }

    @Override
    public boolean goToThirdLevel() {
        return mJoystick.getYButton();
    }
    
    @Override
    public boolean goToFirstLevelBackwards() {
        return mJoystick.getRawAxis(1) < -Constants.kJoystickThreshold && goToFirstLevel();
    }

    @Override
    public boolean goToShip() {
        return mJoystick.getXButton();
    }

    // Elevator
    @Override
    public double getJogElevatorThrottle() {
        return -mJoystick.getRawAxis(5);
    }

    // Wrist
    @Override
    public double getJogWristThrottle() {
        return -mJoystick.getRawAxis(4);
    }

    // Cargo Intake
    @Override
    public boolean getRunIntake() {
        return mJoystick.getTriggerAxis(Hand.kRight) > Constants.kJoystickThreshold;
    }
    
    @Override
    public boolean getRunOuttake() {
        return mJoystick.getTriggerAxis(Hand.kLeft) > Constants.kJoystickThreshold;
    }

    @Override
    public boolean getScoreHatch() {
        return mJoystick.getBumper(Hand.kLeft);
    }

    @Override
    public void setRumble(boolean on) {
        mJoystick.setRumble(RumbleType.kRightRumble, on ? 1.0 : 0.0);
    }

    @Override
    public boolean climbMode() {
        return mJoystick.getBumper(Hand.kLeft) &&  mJoystick.getBumper(Hand.kRight) && 
        (mJoystick.getTriggerAxis(Hand.kLeft) > Constants.kJoystickThreshold) && (mJoystick.getTriggerAxis(Hand.kRight) > Constants.kJoystickThreshold); 
    }

    @Override
    public boolean dropCrawlers() {
            return mJoystick.getAButtonPressed();
    }

    @Override
    public boolean Crawl() {
            return mJoystick.getBButtonPressed();
    }

    @Override
    public boolean exitClimbMode() {
            return mJoystick.getPOV() == 180;
    }


}
