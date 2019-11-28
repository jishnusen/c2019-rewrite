package com.frc1678.c2019.subsystems;

import com.ctre.phoenix.motorcontrol.RemoteFeedbackDevice;
import com.ctre.phoenix.motorcontrol.RemoteSensorSource;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

import com.frc1678.c2019.Constants;
import com.frc1678.c2019.states.SuperstructureConstants;
import com.team254.lib.drivers.MotorChecker;
import com.team254.lib.drivers.TalonSRXChecker;
import com.team254.lib.drivers.TalonSRXUtil;

import java.util.ArrayList;

public class Wrist extends ServoMotorSubsystem {
    private static Wrist mInstance;

    public synchronized static Wrist getInstance() {
        if (mInstance == null) {
            mInstance = new Wrist(Constants.kWristConstants);
        }

        return mInstance;
    }

    private Wrist(final ServoMotorSubsystemConstants constants) {
        super(constants);

        TalonSRXUtil.checkError(mMaster.configRemoteFeedbackFilter(Constants.kCanifierId, RemoteSensorSource.CANifier_Quadrature,
                0, Constants.kLongCANTimeoutMs),
                "Could not set wrist encoder!!!: ");

        TalonSRXUtil.checkError(mMaster.configSelectedFeedbackSensor(
                RemoteFeedbackDevice.RemoteSensor0, 0, Constants.kLongCANTimeoutMs),
                "Could not detect wrist encoder: ");
    }

    public synchronized void setRampRate(double rampRate) {
        mMaster.configClosedloopRamp(rampRate, 0);
    }

    public synchronized double getAngle() {
        return getPosition();
    }

    public boolean getWantsPassThrough() {
        if ((getSetpoint() > SuperstructureConstants.kWristSafeBackwardsAngle
                && getAngle() < SuperstructureConstants.kWristSafeBackwardsAngle)
                || (getSetpoint() < SuperstructureConstants.kWristSafeForwardsAngle
                        && getAngle() > SuperstructureConstants.kWristSafeForwardsAngle)) {
            return true; // Wrist is going to pass through soon
        } else {
            return false;
        }
    }

    @Override
    public boolean checkSystem() {
        return TalonSRXChecker.checkMotors(this,  new ArrayList<MotorChecker.MotorConfig<TalonSRX>>() {
            private static final long serialVersionUID = 7928434738232966358L;
            {
                add(new MotorChecker.MotorConfig<>("wrist_master", mMaster));
            }
        }, new MotorChecker.CheckerConfig() {
            {
                mRunTimeSec = 1.0;
                mRunOutputPercentage = 0.20;

                mRPMFloor = 50.0;
                mCurrentFloor = 2.0;

                mRPMSupplier = () -> mMaster.getSelectedSensorVelocity(0);
            }
        });
    }
}