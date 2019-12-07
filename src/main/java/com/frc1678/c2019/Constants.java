package com.frc1678.c2019;

import com.frc1678.c2019.subsystems.Limelight.LLConstants;
import com.frc1678.c2019.subsystems.ServoMotorSubsystem.ServoMotorSubsystemConstants;
import com.frc1678.c2019.subsystems.ServoMotorSubsystem.TalonSRXConstants;
import edu.wpi.first.wpilibj.Solenoid;
import java.net.NetworkInterface;
import java.net.SocketException;
import java.util.Enumeration;

/**
 * A list of constants used by the rest of the robot code. This include physics constants as well as constants
 * determined through calibrations.
 */
public class Constants {
    public static final double kLooperDt = 0.02;
    public static final boolean kDebuggingOutput = false;
    /* ROBOT PHYSICAL CONSTANTS */
    // Wheels
    public static final double kDriveWheelTrackWidthInches = 27.75;
    public static final double kDriveWheelDiameterInches = 4.0;
    public static final double kDriveWheelRadiusInches = kDriveWheelDiameterInches / 2.0;
    public static final double kTrackScrubFactor = 1.0;  // Tune me!
    // Tuned dynamics
    public static final double kRobotLinearInertia = 60.0;  // kg TODO tune
    public static final double kRobotAngularInertia = 12.0;  // kg m^2 TODO tune
    public static final double kRobotAngularDrag = 0.0;  // N*m / (rad/sec) TODO tune
    public static final double kDriveVIntercept = 0.44;  // V
    public static final double kDriveKv = 0.129;  // V per rad/s
    public static final double kDriveKa = 0.012;  // V per rad/s^2
    public static final double kPathKX = 4.0;  // units/s per unit of error
    public static final double kPathLookaheadTime = 0.4;  // seconds to look ahead along the path for steering
    public static final double kPathMinLookaheadDistance = 24.0;  // inches
    public static final double kPathFollowingMaxAccel = 80.0;  // inches per second ^ 2
    // PID gains for drive velocity loop 
    // Units: setpoint, error, and output are in ticks per second.
    public static final double kDriveVelocityKp = 0.1;
    public static final double kDriveVelocityKi = 0.0;
    public static final double kDriveVelocityKd = 1.0;
    public static final double kDriveVelocityKf = 0.0;
    public static final int kDriveVelocityIZone = 0;
    public static final double kDriveVoltageRampRate = 0.0;
    // PID gains for elevator velocity loop (HIGH GEAR)
    // Units: setpoint, error, and output are in native units per 100ms.
    // Elevator encoder is CTRE mag encoder which is 4096 native units per revolution.

      // elevator
      public static final double kElevatorFeedforward = 1.3 / 12;

      public static final ServoMotorSubsystemConstants kElevatorConstants = new ServoMotorSubsystemConstants();
      static {
          kElevatorConstants.kName = "Elevator";
  
          kElevatorConstants.kMasterConstants.id = 6;
          kElevatorConstants.kMasterConstants.invert_motor = false;
          kElevatorConstants.kMasterConstants.invert_sensor_phase = false;
          kElevatorConstants.kSlaveConstants = new TalonSRXConstants[3];
  
          kElevatorConstants.kSlaveConstants[0] = new TalonSRXConstants();
          kElevatorConstants.kSlaveConstants[1] = new TalonSRXConstants();
          kElevatorConstants.kSlaveConstants[2] = new TalonSRXConstants();
  
          kElevatorConstants.kSlaveConstants[0].id = 7;
          kElevatorConstants.kSlaveConstants[0].invert_motor = false;
          kElevatorConstants.kSlaveConstants[1].id = 8;
          kElevatorConstants.kSlaveConstants[1].invert_motor = false;
          kElevatorConstants.kSlaveConstants[2].id = 9;
          kElevatorConstants.kSlaveConstants[2].invert_motor = false;
  
          // Unit == Inches
          kElevatorConstants.kHomePosition = 0.0;  // Inches off ground
          kElevatorConstants.kTicksPerUnitDistance = (4096) / (Math.PI * 1.25 * 1.6);
          kElevatorConstants.kKp = 0.12;
          kElevatorConstants.kKi = 0;
          kElevatorConstants.kKd = 4.0;
          kElevatorConstants.kKf = .06; // lower speed:  0.08;
          kElevatorConstants.kMaxIntegralAccumulator = 500000;
          kElevatorConstants.kIZone = 0; // Ticks
          kElevatorConstants.kDeadband = 0; // Ticks
  
          kElevatorConstants.kPositionKp = 0.1;
          kElevatorConstants.kPositionKi = 0;
          kElevatorConstants.kPositionKd = 3.0;
          kElevatorConstants.kPositionKf = 0;
         // kElevatorConstants.kPositionMaxIntegralAccumulator = 0;
          kElevatorConstants.kPositionIZone = 0; // Ticks
          kElevatorConstants.kPositionDeadband = 0; // Ticks
  
          kElevatorConstants.kMaxUnitsLimit = 71; // inches
          kElevatorConstants.kMinUnitsLimit = 0.0; // inches
  
          kElevatorConstants.kCruiseVelocity = 14325; // Ticks / 100ms
          kElevatorConstants.kAcceleration = 19100; // Ticks / 100ms / s
          kElevatorConstants.kRampRate = 0.1; // s
          kElevatorConstants.kContinuousCurrentLimit = 20; // amps
          kElevatorConstants.kPeakCurrentLimit = 35; // amps
          kElevatorConstants.kPeakCurrentDuration = 200; // milliseconds
      }

    // wrist
    public static final double kAutoWristRampRate = 0.01;

    // wrist
    // PID gains for wrist velocity loop
    public static final double kWristKp = 0.1;
    public static final double kWristKi = 0.0;
    public static final double kWristKd = 0.0;
    public static final double kWristKf = 0.0465;
    public static final double kWristJogKp = 2.0 / 40;
    public static final double kWristJogKd = 40.0 / 40;
    public static final double kWristKaWithCube = 0.006;
    public static final double kWristKaWithoutCube = 0.003;
    public static final double kWristKfMultiplierEmpty = 0.1275;
    public static final double kWristElevatorAccelerationMultiplier = -1.0;
    public static final double kWristEpsilon = 2.0;
    public static final int kWristMaxIntegralAccumulator = 500000; //todo: tune me
    public static final int kWristIZone = 500; //todo: tune me
    public static final int kWristDeadband = 5; //todo: tune me
    public static final int kWristCruiseVelocity = 20500; //todo: tune me
    public static final int kWristAcceleration = 25000; //2000 //todo: tune me
    public static final double kWristRampRate = 0.001;
    public static final int kWristMasterId = 14;
    /* I/O */
    // (Note that if multiple talons are dedicated to a mechanism, any sensors
    // are attached to the master)
    public static final int kCANTimeoutMs = 10; //use for on the fly updates
    public static final int kLongCANTimeoutMs = 100; //use for constructors
    public static final int kCameraStreamPort = 5810;
    // Drive
    public static final int kLeftDriveMasterId = 1;
    public static final int kLeftDriveSlaveAId = 2;
    public static final int kLeftDriveSlaveBId = 3;
    public static final int kRightDriveMasterId = 12;
    public static final int kRightDriveSlaveAId = 11;
    public static final int kRightDriveSlaveBId = 10;
    // Intakes
    public static final int kCanifierId = 0;
    // Elevator
    public static final int kElevatorMasterId = 6;
    public static final int kElevatorRightSlaveId = 7;
    public static final int kElevatorLeftSlaveAId = 8;
    public static final int kElevatorLeftSlaveBId = 9;
    public static final int kCrawlerId = 15; // TODO set correct number
    // Cargo Intake
    public static final int kCargoIntakeRollerId = 4;
    // Solenoids
    public static final int kElevatorShifterSolenoidId = 0;
    public static final int kBackplateSolenoidId = 2;
    public static final int kArrowheadSolenoidId = 3;
    public static final int kCargoIntakePopoutSolenoidId = 6;
    public static final int kPinsSolenoidId = 1; // TODO  find correct port numbers
    public static final int kForksSolenoidId = 5; // TODO find correct port numbers
    public static final int kDropSolenoidId = 4; // TODO find correct port numbers
    
    // Control Board
    public static final boolean kUseGamepadForDriving = false;
    public static final boolean kUseGamepadForButtons = true;
    
    public static final int kDriveGamepadPort = 0;
    public static final int kButtonGamepadPort = 2;
    public static final int kMainThrottleJoystickPort = 1;
    public static final int kMainTurnJoystickPort = 0;
    public static final double kJoystickThreshold = 0.5;
    public static final double kJoystickJogThreshold = 0.4;


    // Limelight

    public static final double kMaxTrackerDistance = 9.0;
    public static final double kMaxGoalTrackAge = 2.5;
    public static final double kMaxGoalTrackSmoothingTime = 0.5;
    public static final double kCameraFrameRate = 90.0;

    public static final LLConstants kTopLimelightConstants = new LLConstants();
    static {
        kTopLimelightConstants.kName = "Front Limelight";
        kTopLimelightConstants.kTableName = "limelight-front";
        kTopLimelightConstants.kLLHeight = 45;  // inches
        kTopLimelightConstants.kObjectHeight = 29;
        kTopLimelightConstants.kCargoObjectHeight = 37;
        kTopLimelightConstants.kLLAngle = -30; // TODO find units
    }

    // Bottom limelight
    public static final LLConstants kBottomLimelightConstants = new LLConstants();
    static {
        kBottomLimelightConstants.kName = "Bottom Limelight";
        kBottomLimelightConstants.kTableName = "limelight-bottom";
        kBottomLimelightConstants.kLLHeight = 6;  // inches
        kBottomLimelightConstants.kObjectHeight = 29;
        kBottomLimelightConstants.kCargoObjectHeight = 37;
        kBottomLimelightConstants.kLLAngle = 40; // TODO find units
    }

    public static Solenoid makeSolenoidForId(int solenoidId) {
        if (solenoidId < 8) {
            return new Solenoid(solenoidId);
        }
        throw new IllegalArgumentException("Solenoid ID not valid: " + solenoidId);
    }
    /**
     * @return the MAC address of the robot
     */
    public static String getMACAddress() {
        try {
            Enumeration<NetworkInterface> nwInterface = NetworkInterface.getNetworkInterfaces();
            StringBuilder ret = new StringBuilder();
            while (nwInterface.hasMoreElements()) {
                NetworkInterface nis = nwInterface.nextElement();
                if (nis != null) {
                    byte[] mac = nis.getHardwareAddress();
                    if (mac != null) {
                        for (int i = 0; i < mac.length; i++) {
                            ret.append(String.format("%02X%s", mac[i], (i < mac.length - 1) ? "-" : ""));
                        }
                        return ret.toString();
                    } else {
                        System.out.println("Address doesn't exist or is not accessible");
                    }
                } else {
                    System.out.println("Network Interface for the specified address is not found.");
                }
            }
        } catch (SocketException e) {
            e.printStackTrace();
        } catch (NullPointerException e) {
            e.printStackTrace();
        }
        return "";
    }
}
