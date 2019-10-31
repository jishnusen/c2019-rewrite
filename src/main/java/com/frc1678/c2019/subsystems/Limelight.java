package com.frc1678.c2019.subsystems;

import com.frc1678.c2019.Constants;
import com.team254.lib.util.Util;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import java.util.ArrayList;
import java.util.Arrays;
import java.util.Comparator;
import java.util.List;


public class Limelight extends Subsystem {

    private PeriodicIO mPeriodicIO = new PeriodicIO();
    private LLConstants mConstants = null;
    private boolean mUpdateOutputs= true;
    private double mTargetDist;
    private double mLatencyCounter;

    public static class LLConstants {
        public String kName = "";
        public String kTableName = "";
        public double kObjectHeight = 0.0;
        public double kLLAngle = 0.0;
        public double kLLHeight = 0.0;
    }

    private NetworkTable mNetworkTable;
    
    public static class PeriodicIO {
        public double latency;
        public int givenLedMode;
        public double xOffset;
        public double yOffset;
        public double skew;
        public boolean hasTarget;
        public boolean limelightOK;

        // OUTPUTS
        public int ledMode = 1; // 0 - use pipeline mode, 1 - off, 2 - blink, 3 - on
        public int camMode = 0; // 0 - vision processing, 1 - driver camera
        public int stream = 2; // sets stream layout if another webcam is attached
        public int snapshot = 0;
    } 

    public enum LedMode {
        PIPELINE, OFF, BLINK, ON
    }

    public synchronized void setLed(LedMode mode) {
        if (mode.ordinal() != mPeriodicIO.ledMode) {
            mPeriodicIO.ledMode = mode.ordinal();
            mUpdateOutputs = true;  
        }
    }

    @Override
    public synchronized void readPeriodicInputs() {
     //   mPeriodicIO.latency = mNetworkTable.getEntry("tl").getDouble(0) / 1000.0 + Constants.kImageCaptureLatency;
        mPeriodicIO.xOffset = mNetworkTable.getEntry("tx").getDouble(0.0);
        mPeriodicIO.skew = mNetworkTable.getEntry("ts").getDouble(0.0);
        mPeriodicIO.givenLedMode = (int) mNetworkTable.getEntry("ledMode").getDouble(0.0);
        mPeriodicIO.yOffset = mNetworkTable.getEntry("ty").getDouble(0.0);
        mPeriodicIO.hasTarget = mNetworkTable.getEntry("tv").getDouble(0.0) == 1.0;
        final double latency = mNetworkTable.getEntry("tl").getDouble(0.0);

        if (mPeriodicIO.latency == latency) {
            mLatencyCounter += 1;
        } else {
            mLatencyCounter = 0;
        }

        mPeriodicIO.latency = latency;
        mPeriodicIO.limelightOK = mLatencyCounter < 10;

        calculateTargetDist();
    }

    @Override
    public synchronized void writePeriodicOutputs() {
        if (mUpdateOutputs) {
            mNetworkTable.getEntry("ledMode").setNumber(mPeriodicIO.ledMode); 
        }
        
    }

    @Override
    public void stop() {}

    @Override
    public boolean checkSystem() {
        return true;
    }

    @Override
    public synchronized void outputTelemetry() {
        SmartDashboard.putBoolean(mConstants.kName + ": Has Target", mPeriodicIO.hasTarget);
        SmartDashboard.putNumber(mConstants.kName + ": XOffset", mPeriodicIO.xOffset);
        SmartDashboard.putNumber(mConstants.kName + ": yOffset", mPeriodicIO.yOffset);
        SmartDashboard.putNumber(mConstants.kName + ": Distance", mTargetDist);

    }
    public double getXOffset() {
        return mPeriodicIO.xOffset;
    }

    private double calculateTargetDist() {
        double delta_h = mConstants.kObjectHeight - mConstants.kLLHeight;
        mTargetDist = Math.abs(delta_h / Math.tan(Math.toRadians(mConstants.kLLAngle + mPeriodicIO.yOffset)));
 
        return mTargetDist;

    }

    public double getTargetDist() {
        return mTargetDist;
    }

    public boolean getHasTarget() {
        return mPeriodicIO.hasTarget;
    }

    public boolean getLimelightOK() {
        return mPeriodicIO.limelightOK;
    }

    public Limelight(LLConstants constants) {
        mConstants = constants;
        mNetworkTable = NetworkTableInstance.getDefault().getTable(constants.kTableName);
    }

    public double getLatency() {
       return mPeriodicIO.latency;
    }


}