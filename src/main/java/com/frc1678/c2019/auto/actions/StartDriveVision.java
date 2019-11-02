package com.frc1678.c2019.auto.actions;

import com.frc1678.c2019.RobotState;
import com.frc1678.c2019.subsystems.Drive;
import com.frc1678.c2019.subsystems.LimelightManager;
import com.frc1678.lib.control.PIDController;
import com.team254.lib.util.DriveSignal;
import com.team254.lib.util.Util;
import edu.wpi.first.wpilibj.Timer;

public class StartDriveVision implements Action {
    private static final Drive mDrive = Drive.getInstance();

    private double mStartTime;
    private boolean useBottomLimelight;
    private  double  mLeft, mRight;
    
    private final LimelightManager mLLManager = LimelightManager.getInstance();
    private final PIDController throttlePID = new PIDController(.15, 0.00, 0.0);
    private final PIDController throttlePID2 = new PIDController(.15, 0.00, 0.0);
    private final PIDController steeringPID = new PIDController(.2, 0.00, 0.01);

  
    public StartDriveVision(boolean top) {
        useBottomLimelight = false;
    }


    @Override
    public boolean isFinished() {
        if (steeringPID.isDone() && (throttlePID2.isDone() || throttlePID.isDone)) {
            return true;
        } else {
            return false;
        }
    }

    @Override
    public void start() {
        mDrive.setOpenLoop(new DriveSignal(mLeft, mRight));
        mStartTime = Timer.getFPGATimestamp();
    }

  
    @Override
    public void update() {

        throttlePID.setGoal(25.0);
        throttlePID2.setGoal(14.0);
        steeringPID.setGoal(0.0);

        steeringPID.reset();
        throttlePID.reset();
        throttlePID2.reset();

        double leftVoltage;
        double rightVoltage;

        double throttle = throttlePID.update(Timer.getFPGATimestamp(), mLLManager.getTargetDist());
        double throttle2 = throttlePID2.update(Timer.getFPGATimestamp(), mLLManager.getTargetDist());
        double steering = steeringPID.update(Timer.getFPGATimestamp(), mLLManager.getXOffset());

        if (!useBottomLimelight) {
            leftVoltage = (throttle + steering) / 12.0;
            rightVoltage = (throttle - steering) / 12.0;
          } else {
            leftVoltage = (throttle2 + steering) / 12.0;
            rightVoltage = (throttle2 - steering) / 12.0;
  
          }

        
        Util.limit(rightVoltage, 1.0);
        Util.limit(leftVoltage, 1.0);

    }

    @Override
    public void done() {
        mDrive.setOpenLoop(new DriveSignal(0.0, 0.0));
    }
}

