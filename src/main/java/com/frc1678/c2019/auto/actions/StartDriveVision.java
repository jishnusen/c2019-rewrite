package com.frc1678.c2019.auto.actions;

import com.frc1678.c2019.RobotState;
import com.frc1678.c2019.subsystems.Drive;
import com.frc1678.c2019.subsystems.LimelightManager;
import com.frc1678.lib.control.PIDController;
import com.team254.lib.util.DriveSignal;
import com.team254.lib.util.Util;
import edu.wpi.first.wpilibj.Timer;

public class StartDriveVision implements Action {
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
        Drive.getInstance().updateVisionPID(true);
    }

  
    @Override
    public void update() {
        Drive.getInstance().updateVisionPID(false);
    }

    @Override
    public void done() {
        Drive.getInstance().setOpenLoop(new DriveSignal(0.0, 0.0));
    }
}

