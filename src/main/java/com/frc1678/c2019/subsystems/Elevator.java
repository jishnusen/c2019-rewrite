package com.frc1678.c2019.subsystems;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.DemandType;
import com.ctre.phoenix.motorcontrol.LimitSwitchNormal;
import com.ctre.phoenix.motorcontrol.LimitSwitchSource;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;
import com.frc1678.c2019.Constants;
import com.team254.lib.drivers.MotorChecker;
import com.team254.lib.drivers.TalonSRXChecker;
import com.team254.lib.drivers.TalonSRXUtil;
import com.team254.lib.util.LatchedBoolean;

import java.util.ArrayList;

public class Elevator extends ServoMotorSubsystem {
    private static Elevator mInstance;
    private boolean mHoming = false;
    private boolean mHasBeenZeroed = false;
    private LatchedBoolean mJustReset = new LatchedBoolean();
    private boolean mCanHome = true;

    public synchronized static Elevator getInstance() {
        if (mInstance == null) {
            mInstance = new Elevator(Constants.kElevatorConstants);
        }
        return mInstance;
    }

    private Elevator(final ServoMotorSubsystemConstants constants) {
        super(constants);
    }

    @Override
    public void stop() {
        setOpenLoop(0.0);
    }

    @Override
    public synchronized void zeroSensors() {
        mMaster.setSelectedSensorPosition(0, 0, 10);
        mHasBeenZeroed = true;
    }

    public synchronized boolean hasBeenZeroed() {
        return mHasBeenZeroed;
    }  

    public synchronized void setCanHome(boolean canHome) {
        mCanHome = canHome;
    }

    public TalonSRX getPigeonTalon() {
       return mSlaves[0]; //right slave
    }

    @Override
    public synchronized boolean atHomingLocation() {
        return mMaster.getSensorCollection().isRevLimitSwitchClosed();
    }

    @Override
    public synchronized void handleMasterReset(boolean reset) {
        if (mJustReset.update(reset) && mCanHome) {
            System.out.println("Elevator going into home mode!");
            mHoming = true;
            mMaster.overrideSoftLimitsEnable(false);
        }
    }

    public synchronized boolean isHoming() {
        return mHoming;
    }

    @Override
    public synchronized void writePeriodicOutputs() {
        if (mHoming) {
                if (atHomingLocation()) {
                        zeroSensors();
                        mMaster.overrideSoftLimitsEnable(true);
                        System.out.println("Homed!!!");
                        mHoming = false;
                }

                if (mControlState == ControlState.OPEN_LOOP) {
                        mMaster.set(ControlMode.PercentOutput, mPeriodicIO.demand, DemandType.ArbitraryFeedForward,
                                0.0);
                } else {
                        mMaster.set(ControlMode.PercentOutput, 0.0, DemandType.ArbitraryFeedForward,
                                0.0);
                }
        } else {
        super.writePeriodicOutputs();
        }
    }


    public synchronized void updateSoftLimit(int limit) {
        mMaster.configForwardSoftLimitThreshold(limit);
    }

    public synchronized void removeCurrentLimits() {
        mMaster.enableCurrentLimit(false);
    }

    @Override
    public boolean checkSystem() {
        boolean leftSide =
                TalonSRXChecker.checkMotors(this,
                new ArrayList<MotorChecker.MotorConfig<TalonSRX>>() {
                            private static final long serialVersionUID = -2218871869023990636L;
			    {
                                add(new MotorChecker.MotorConfig<>("left slave 1", mSlaves[1]));
                                add(new MotorChecker.MotorConfig<>("left slave 2", mSlaves[2]));
                            }
                        }, new MotorChecker.CheckerConfig() {
                            {
                                mCurrentFloor = 2;
                                mRPMFloor = 200;
                                mCurrentEpsilon = 2.0;
                                mRPMEpsilon = 250;
                                mRunTimeSec = 2.0;
                                mRunOutputPercentage = -0.4;
                                mRPMSupplier = () -> -mMaster.getSelectedSensorVelocity(0);
                            }
                        });
        boolean rightSide =
                TalonSRXChecker.checkMotors(this,
                new ArrayList<MotorChecker.MotorConfig<TalonSRX>>() {
                           private static final long serialVersionUID = 3746808535371453536L;
                           {
                                add(new MotorChecker.MotorConfig<>("master", mMaster));
                                add(new MotorChecker.MotorConfig<>("right slave", mSlaves[0]));
                        }
                        }, new TalonSRXChecker.CheckerConfig() {
                            {
                                mCurrentFloor = 2;
                                mRPMFloor = 200;
                                mCurrentEpsilon = 2.0;
                                mRPMEpsilon = 250;
                                mRunTimeSec = 2.0;
                                mRunOutputPercentage = -0.4;
                                mRPMSupplier = () -> -mMaster.getSelectedSensorVelocity(0);
                            }
                        });

        return leftSide && rightSide;
    }
}
