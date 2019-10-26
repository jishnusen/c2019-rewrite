package com.frc1678.c2019.controlboard;

import com.frc1678.c2019.Constants;
import edu.wpi.first.wpilibj.Joystick;

public class MainDriveControlBoard implements IDriveControlBoard {
    private static MainDriveControlBoard mInstance = null;

    public static MainDriveControlBoard getInstance() {
        if (mInstance == null) {
            mInstance = new MainDriveControlBoard();
        }
        return mInstance;
    }

    private final Joystick mThrottleStick;
    private final Joystick mTurnStick;

    private MainDriveControlBoard() {
        mThrottleStick = new Joystick(Constants.kMainThrottleJoystickPort);
        mTurnStick = new Joystick(Constants.kMainTurnJoystickPort);
    }

    @Override
    public double getThrottle() {
        return mThrottleStick.getRawAxis(1);
    }

    @Override
    public double getTurn() {
        return -mTurnStick.getRawAxis(0);
    }

    @Override
    public boolean getQuickTurn() {
        return mTurnStick.getRawButton(5);
    }

    @Override
    public boolean getVision() {
        return mThrottleStick.getRawButton(1);
    }

    @Override
    public boolean getStartVision() {
        return mThrottleStick.getRawButtonPressed(1);
    }
}
