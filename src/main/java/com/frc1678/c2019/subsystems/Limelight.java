package com.frc1678.c2019.subsystems;

import com.frc1678.c2019.Constants;
import com.frc1678.c2019.RobotState;
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
    private double mHeading;
    private double mTargetDist;

    public static class LLConstants {
        public String kName = "";
        public String kTableName = "";
        double kObjectHeight = 0.0;
        double kLLAngle = 0.0;
        double kLLKeight = 0.0;
    }

    private NetworkTable mNetworkTable;
    
    public static class PeriodicIO {
        public double latency;
        public int givenLedMode;
        public double xOffset;
        public double yOffset;
        boolean mSeesTarget = false;
        public double skew;
        boolean mToTheLeft;


        // OUTPUTS
        public int ledMode = 1; // 0 - use pipeline mode, 1 - off, 2 - blink, 3 - on
        public int camMode = 0; // 0 - vision processing, 1 - driver camera
        public int stream = 2; // sets stream layout if another webcam is attached
        public int snapshot = 0;
    } 

    public enum LedMode {
        OFF, BLINK, ON
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
        mPeriodicIO.givenLedMode = (int) mNetworkTable.getEntry("ledMode").getDouble(1.0);
        mPeriodicIO.yOffset = mNetworkTable.getEntry("ty").getDouble(0.0);
        mSeesTarget = mNetworkTable.getEntry("tv").getDouble(0) == 1.0;
    }

    @Override
    public synchronized void writePeriodicOutputs() {
        
    }

    @Override
    public void stop() {}

    @Override
    public boolean checkSystem() {
        return true;
    }

    @Override
    public synchronized void outputTelemetry() {
        SmartDashboard.putBoolean(mConstants.kName + ": Has Target", mSeesTarget);
    }
    
    public synchronized void update() {
              
        if (mPeriodicIO.skew > -45) {
            mHeading = Math.abs(mPeriodicIO.skew / 8.);
            mToTheLeft = true;
        } else {
            mHeading = Math.abs((mPeriodicIO.skew + 90) / 8.);
            mToTheLeft = true;
        }

        mTargetDist = Math.tan(mPeriodicIO.yOffset);


    }

    public Limelight(LLConstants constants) {
        mConstants = constants;
        mNetworkTable = NetworkTableInstance.getDefault().getTable(constants.kTableName);
    }

    public double getLatency() {
       // return mPeriodicIO.latency;
    }


}