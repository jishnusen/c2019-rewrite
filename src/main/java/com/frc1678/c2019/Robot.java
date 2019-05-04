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
import com.frc1678.c2019.statemachines.HatchIntakeStateMachine.WantedAction;
import com.frc1678.c2019.subsystems.RobotStateEstimator;
import com.frc1678.c2019.states.SuperstructureConstants;
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
                        Superstructure.getInstance(),
                        HatchIntake.getInstance(),
                        CargoIntake.getInstance(),
                        Wrist.getInstance(),
                        Elevator.getInstance(),
                        CarriageCanifier.getInstance(),
                        Infrastructure.getInstance(),
                        Limelight.getInstance()//,
        )
    );

    private Drive mDrive = Drive.getInstance();
    private Limelight mLimelight = Limelight.getInstance();
    private HatchIntake mHatchIntake = HatchIntake.getInstance();
    private CargoIntake mCargoIntake = CargoIntake.getInstance();
    private Wrist mWrist = Wrist.getInstance();
    private Infrastructure mInfrastructure = Infrastructure.getInstance();
    private Superstructure mSuperstructure = Superstructure.getInstance();
    private Elevator mElevator = Elevator.getInstance();

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
                mElevator.resetIfAtLimit();
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

            final boolean cargo_preset = mCargoIntake.hasCargo();
            double height = 0.0;
            double angle = 0.0;
            if (mControlBoard.goToGround()) {
                height = SuperstructureConstants.kGroundHeight;
                angle = SuperstructureConstants.kGroundAngle;
                mHatchIntake.setState(WantedAction.PREP_SCORE);
            } else if (mControlBoard.goToStow()) {
                height = SuperstructureConstants.kStowHeight;
                angle = SuperstructureConstants.kStowAngle;
                mHatchIntake.setState(WantedAction.PREP_SCORE);
            } else if (mControlBoard.goToShip()) {
                height = cargo_preset ? SuperstructureConstants.kCargoShipForwardsHeight : 
                    SuperstructureConstants.kHatchShipForwardsHeight;
                angle = cargo_preset ? SuperstructureConstants.kCargoShipForwardsAngle : 
                    SuperstructureConstants.kHatchForwardsAngle;
                mHatchIntake.setState(WantedAction.PREP_SCORE);
            } else if (mControlBoard.goToFirstLevel()) {
                height = cargo_preset ? SuperstructureConstants.kCargoRocketFirstHeight : SuperstructureConstants.kHatchRocketFirstHeight;
                angle = cargo_preset ? SuperstructureConstants.kCargoRocketFirstAngle : SuperstructureConstants.kHatchForwardsAngle;
                mHatchIntake.setState(WantedAction.PREP_SCORE);
            } else if (mControlBoard.goToSecondLevel()) {
                height = cargo_preset ? SuperstructureConstants.kCargoRocketSecondHeight : SuperstructureConstants.kHatchRocketSecondHeight;
                angle = cargo_preset ? SuperstructureConstants.kCargoRocketSecondAngle : SuperstructureConstants.kHatchForwardsAngle;
                mHatchIntake.setState(WantedAction.PREP_SCORE);
            } else if (mControlBoard.goToThirdLevel()) {
                height = cargo_preset ? SuperstructureConstants.kCargoRocketThirdHeight : SuperstructureConstants.kHatchRocketThirdHeight;
                angle = cargo_preset ? SuperstructureConstants.kCargoRocketThirdAngle : SuperstructureConstants.kHatchForwardsAngle;
                mHatchIntake.setState(WantedAction.PREP_SCORE);
            } else if (mControlBoard.goToFirstLevelBackwards() && !cargo_preset) {
                height = SuperstructureConstants.kHatchRocketBackwardsHeight;
                angle = SuperstructureConstants.kHatchBackwardsAngle;
                mHatchIntake.setState(WantedAction.PREP_SCORE);
            }

            if (mControlBoard.getScoreHatch()) {
                mHatchIntake.setState(WantedAction.SCORE);
            }
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
