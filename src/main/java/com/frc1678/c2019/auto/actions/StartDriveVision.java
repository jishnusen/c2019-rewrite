package com.frc1678.c2019.auto.actions;

import com.frc1678.c2019.RobotState;
import com.frc1678.c2019.subsystems.Drive;
import com.frc1678.c2019.subsystems.LimelightManager;
import com.frc1678.lib.control.PIDController;
import com.team254.lib.util.DriveSignal;
import com.team254.lib.util.Util;
import edu.wpi.first.wpilibj.Timer;

public class StartDriveVision implements Action {
    private  double endTargetDist;
    
    private final LimelightManager mLLManager = LimelightManager.getInstance();
  
    public StartDriveVision(double endDistance) {
        endTargetDist = endDistance;
    }


    @Override
    public boolean isFinished() {
        if (mLLManager.getActiveLimelightObject().getXOffset() < 1.7 && mLLManager.getActiveLimelightObject().getTargetDist() < endTargetDist) {
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

