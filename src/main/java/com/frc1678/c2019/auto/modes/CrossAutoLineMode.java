package com.frc1678.c2019.auto.modes;

import com.frc1678.c2019.auto.AutoModeBase;
import com.frc1678.c2019.auto.AutoModeEndedException;
import com.frc1678.c2019.auto.actions.*;
import com.frc1678.c2019.paths.TrajectoryGenerator;

public class CrossAutoLineMode extends AutoModeBase {
    private static final TrajectoryGenerator mTrajectoryGenerator = TrajectoryGenerator.getInstance();

    private DriveTrajectory mCrossLine = new DriveTrajectory(mTrajectoryGenerator.getTrajectorySet().crossAutoLine.right, true);

    @Override
    protected void routine() throws AutoModeEndedException {
        System.out.println("Running Cross auto line");
        runAction(mCrossLine);
    }
}
