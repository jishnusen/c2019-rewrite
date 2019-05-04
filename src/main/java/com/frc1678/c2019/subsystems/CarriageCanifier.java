package com.frc1678.c2019.subsystems;

import com.ctre.phoenix.CANifier;
import com.ctre.phoenix.CANifierStatusFrame;
import com.frc1678.c2019.Constants;

public class CarriageCanifier extends Subsystem {
    private static CarriageCanifier mInstance;
    private CANifier mCanifier;
    private PeriodicInputs mPeriodicInputs;

    private CarriageCanifier() {
        mCanifier = new CANifier(Constants.kCanifierId);
        mCanifier.setStatusFramePeriod(CANifierStatusFrame.Status_1_General, 100, Constants.kLongCANTimeoutMs);
        mCanifier.setStatusFramePeriod(CANifierStatusFrame.Status_2_General, 2, Constants.kLongCANTimeoutMs);
        mPeriodicInputs = new PeriodicInputs();
    }

    public synchronized static CarriageCanifier getInstance() {
        if (mInstance == null) {
            mInstance = new CarriageCanifier();
        }
        return mInstance;
    }

    public int getWristTicks() {
        return mCanifier.getQuadraturePosition();
    }

    public synchronized boolean getCargoProxy() {
        return mPeriodicInputs.cargo_proxy_;
    }

    public synchronized boolean getLeftHatchProxy() {
        return mPeriodicInputs.left_hatch_proxy_;
    }

    public synchronized boolean getRightHatchProxy() {
        return mPeriodicInputs.right_hatch_proxy_;
    }

    public synchronized boolean getLimR() {
        return mPeriodicInputs.limr_;
    }

    public synchronized void resetWristEncoder() {
        mCanifier.setQuadraturePosition(0, 0 );
    }

    public int getDeviceId() {
        return mCanifier.getDeviceID();
    }

    @Override
    public synchronized void readPeriodicInputs() {
        CANifier.PinValues pins = new CANifier.PinValues();
        mCanifier.getGeneralInputs(pins);
        mPeriodicInputs.cargo_proxy_ = pins.SPI_CLK_PWM0;
        mPeriodicInputs.left_hatch_proxy_ = pins.SPI_MOSI_PWM1;
        mPeriodicInputs.right_hatch_proxy_ =  pins.SPI_CS_PWM3;
        mPeriodicInputs.limr_ = !pins.LIMR;
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
        mPeriodicInputs = new PeriodicInputs();
    }

    private static class PeriodicInputs {
        public boolean cargo_proxy_;
        public boolean left_hatch_proxy_;
        public boolean right_hatch_proxy_;
        public boolean limr_;
    }
}
