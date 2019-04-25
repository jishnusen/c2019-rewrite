/*----------------------------------------------------------------------------*/
/* Copyright (c) 2017-2018 FIRST. All Rights Reserved.                        */
/* Open Source Software - may be modified and shared by FRC teams. The code   */
/* must be accompanied by the FIRST BSD license file in the root directory of */
/* the project.                                                               */
/*----------------------------------------------------------------------------*/

package com.frc1678.c2019;

import com.frc1678.c2019.auto.AutoModeBase;
import com.frc1678.c2019.auto.AutoModeExecuter;
import com.frc1678.c2019.loops.Looper;
import com.frc1678.c2019.paths.TrajectoryGenerator;
import com.frc1678.c2019.subsystems.RobotStateEstimator;
//import com.frc1678.c2019.statemachines.IntakeStateMachine;
//import com.team254.frc2018.statemachines.SuperstructureStateMachine;
//import com.team254.frc2018.states.SuperstructureConstants;
import com.frc1678.c2019.subsystems.*;
import com.team254.lib.geometry.Pose2d;
import com.team254.lib.util.*;
import edu.wpi.cscore.MjpegServer;
import edu.wpi.cscore.UsbCamera;
import edu.wpi.cscore.VideoMode;
import edu.wpi.first.wpilibj.CameraServer;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.IterativeRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import java.util.Arrays;
import java.util.Optional;

public class Robot extends IterativeRobot {
  private Looper mEnabledLooper = new Looper();
  private Looper mDisabledLooper = new Looper();
  private CheesyDriveHelper mCheesyDriveHelper = new CheesyDriveHelper();
  private IControlBoard mControlBoard = ControlBoard.getInstance();
  private TrajectoryGenerator mTrajectoryGenerator = TrajectoryGenerator.getInstance();
  private AutoModeSelector mAutoModeSelector = new AutoModeSelector();

  private final SubsystemManager mSubsystemManager = new SubsystemManager(
    Arrays.asList(
            RobotStateEstimator.getInstance(),
            Drive.getInstance(),
//            Superstructure.getInstance(),
//            Intake.getInstance(),
//            Wrist.getInstance(),
//            Elevator.getInstance(),
//            CarriageCanifier.getInstance(),
//            Infrastructure.getInstance(),
            Limelight.getInstance()//,
//            CheesyVision2.getInstance()
    )
  );

  private Drive mDrive = Drive.getInstance();
  private Limelight mLimelight = Limelight.getInstance();

  private AutoModeExecuter mAutoModeExecuter;

  public Robot() {
    CrashTracker.logRobotConstruction();
    mTrajectoryGenerator.generateTrajectories();

  }

  @Override
  public void robotInit() {
    try {
      //init camera stream
      UsbCamera camera = CameraServer.getInstance().startAutomaticCapture();
      camera.setVideoMode(VideoMode.PixelFormat.kMJPEG, 320, 240, 15);
      MjpegServer cameraServer = new MjpegServer("serve_USB Camera 0", Constants.kCameraStreamPort);
      cameraServer.setSource(camera);

      CrashTracker.logRobotInit();

      mSubsystemManager.registerEnabledLoops(mEnabledLooper);
      mSubsystemManager.registerDisabledLoops(mDisabledLooper);

      mTrajectoryGenerator.generateTrajectories();
    } catch (Throwable t) {
      CrashTracker.logThrowableCrash(t);
      throw t;
    }
  }

  @Override
  public void disabledInit() {
    SmartDashboard.putString("Match Cycle", "DISABLED");
    try {
        CrashTracker.logDisabledInit();
        mEnabledLooper.stop();

        Drive.getInstance().zeroSensors();
        RobotState.getInstance().reset(Timer.getFPGATimestamp(), Pose2d.identity());

        mDisabledLooper.start();
    } catch (Throwable t) {
        CrashTracker.logThrowableCrash(t);
        throw t;
    }

  }

  @Override
  public void autonomousInit() {
    SmartDashboard.putString("Match Cycle", "AUTONOMOUS");

    try {
        CrashTracker.logAutoInit();
        mDisabledLooper.stop();

        RobotState.getInstance().reset(Timer.getFPGATimestamp(), Pose2d.identity());

        Drive.getInstance().zeroSensors();

        mAutoModeExecuter.start();

        mEnabledLooper.start();
    } catch (Throwable t) {
        CrashTracker.logThrowableCrash(t);
        throw t;
    }
  }

  @Override
  public void teleopInit() {
    SmartDashboard.putString("Match Cycle", "TELEOP");

    try {
        CrashTracker.logTeleopInit();
        mDisabledLooper.stop();

        RobotState.getInstance().reset(Timer.getFPGATimestamp(), Pose2d.identity());
        mEnabledLooper.start();

        mDrive.setVelocity(DriveSignal.NEUTRAL, DriveSignal.NEUTRAL);
        mDrive.setOpenLoop(new DriveSignal(0.05, 0.05));
    } catch (Throwable t) {
        CrashTracker.logThrowableCrash(t);
        throw t;
    }
  }

  @Override
  public void testInit() {
    SmartDashboard.putString("Match Cycle", "TEST");

    try {
        System.out.println("Starting check systems.");

        mDisabledLooper.stop();
        mEnabledLooper.stop();

        mDrive.checkSystem();
        //mIntake.checkSystem();
        //mWrist.checkSystem();
        //mElevator.checkSystem();

    } catch (Throwable t) {
        CrashTracker.logThrowableCrash(t);
        throw t;
    }
  }

  @Override
  public void disabledPeriodic() {
    SmartDashboard.putString("Match Cycle", "DISABLED");

    mLimelight.setStream(2);

    try {
        outputToSmartDashboard();
        Optional<AutoModeBase> autoMode = mAutoModeSelector.getAutoMode();
        if (autoMode.isPresent() && autoMode.get() != mAutoModeExecuter.getAutoMode()) {
            System.out.println("Set auto mode to: " + autoMode.get().getClass().toString());
            mAutoModeExecuter.setAutoMode(autoMode.get());
        }
        System.gc();
    } catch (Throwable t) {
        CrashTracker.logThrowableCrash(t);
        throw t;
    }
  }

  @Override
  public void autonomousPeriodic() {
    SmartDashboard.putString("Match Cycle", "AUTONOMOUS");

    outputToSmartDashboard();
    try {

    } catch (Throwable t) {
        CrashTracker.logThrowableCrash(t);
        throw t;
    }
  }

  @Override
  public void teleopPeriodic() {
    SmartDashboard.putString("Match Cycle", "TELEOP");
    double timestamp = Timer.getFPGATimestamp();

    double throttle = mControlBoard.getThrottle();
    double turn = mControlBoard.getTurn();

    try {
      mDrive.setOpenLoop(mCheesyDriveHelper.cheesyDrive(throttle, turn, mControlBoard.getQuickTurn(), false));
      outputToSmartDashboard();
    } catch (Throwable t) {
      CrashTracker.logThrowableCrash(t);
      throw t;
    }
  }

  @Override
  public void testPeriodic() {
      SmartDashboard.putString("Match Cycle", "TEST");
  }

  public void outputToSmartDashboard() {
      RobotState.getInstance().outputToSmartDashboard();
      Drive.getInstance().outputTelemetry();
      mEnabledLooper.outputToSmartDashboard();
      // SmartDashboard.updateValues();
  }
}
