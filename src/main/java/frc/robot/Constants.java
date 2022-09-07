// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.math.controller.ElevatorFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import frc.lib.controller.PIDGains;
import frc.lib.util.LinearInterpolatedTable2d;
import frc.lib.util.RobotName;

/**
 * The Constants class provides a convenient place for teams to hold robot-wide numerical or boolean
 * constants. This class should not be used for any other purpose. All constants should be declared
 * globally (i.e. public static). Do not put anything functional in this class.
 *
 * <p>It is advised to statically import this class (or one of its inner classes) wherever the
 * constants are needed, to reduce verbosity.
 */
public final class Constants {
  public static final class Drivetrain {
    public static final int kFrontLeftDriveCanId = 5;
    public static final int kRearLeftDriveCanId = 7;
    public static final int kFrontRightDriveCanId = 3;
    public static final int kRearRightDriveCanId = 1;

    public static final int kFrontLeftTurningCanId = 6;
    public static final int kRearLeftTurningCanId = 8;
    public static final int kFrontRightTurningCanId = 4;
    public static final int kRearRightTurningCanId = 2;

    // Drive motor current limit in Amps
    public static final int kDriveMotorCurrentLimit = 50;
    // public static final int kTurningMotorCurrentLimit = 25;
    public static final PIDGains kTurningMotorPIDGains = new PIDGains(8.0, 0.00, 0.3);

    // Max Velocity in m/s
    public static final double kTurningMotorMaxVelocity = 5.0;

    // Max acceleration in m/s^2
    public static final double kTurningMotorMaxAccel = 5.0;
    public static final double kMaxDriveSpeed = 5.0;
    public static PIDGains kDriveMotorPIDGains = new PIDGains(0.04, 0.0, 0.0);

    public static final double kTrackWidth = Units.inchesToMeters(26.0);
    public static final double kWheelBase = Units.inchesToMeters(26.0);
    public static final SwerveDriveKinematics kDriveKinematics =
        new SwerveDriveKinematics(
            new Translation2d(kWheelBase / 2, kTrackWidth / 2),
            new Translation2d(kWheelBase / 2, -kTrackWidth / 2),
            new Translation2d(-kWheelBase / 2, kTrackWidth / 2),
            new Translation2d(-kWheelBase / 2, -kTrackWidth / 2));

    // Number of rotations of the motor / number of rotations of the output
    public static final double kDriveMotorReduction = 8.31 / 1.0;

    // Theory = 0.1016, measured by driving the robot a fixed distance: ~0.120
    // Something doesn't add up here...
    public static final double kWheelDiameterMeters = 0.1016; // 0.120;

    // Assumes the encoders are directly mounted on the wheel shafts
    public static final double kDriveEncoderPositionFactor =
        (kWheelDiameterMeters * Math.PI) / (double) kDriveMotorReduction;

    public static final double kDriveEncoderVelocityFactor =
        // Assumes the encoders are directly mounted on the wheel shafts
        ((kWheelDiameterMeters * Math.PI) / (double) kDriveMotorReduction) / 60.0;
    public static final boolean kDriveMotorInvert = false;

    public static final double kTurningModuleGearRatio = 31752.0 / 841.0;
    public static final double kTurningEncoderPositionFactor =
        (2 * Math.PI) / kTurningModuleGearRatio;
    public static final double kTurningEncoderVelocityFactor =
        ((2 * Math.PI) / kTurningModuleGearRatio) / 60.0;
    public static final boolean kTurningMotorInvert = true;
    public static final SimpleMotorFeedforward kDriveFeedforward =
        new SimpleMotorFeedforward(0.0467, 3.3076, 0.01897);
    public static final SimpleMotorFeedforward kTurningFeedforward =
        new SimpleMotorFeedforward(0.35233, 0.39185, 0.0058658);
    public static final TrapezoidProfile.Constraints kTurningConstraints =
        new TrapezoidProfile.Constraints(20, 200);
    public static final int kFrontLeftAbsoluteEncoderPort = 0;
    public static final int kFrontRightAbsoluteEncoderPort = 3;
    public static final int kRearLeftAbsoluteEncoderPort = 1;
    public static final int kRearRightAbsoluteEncoderPort = 2;
    public static final double kFrontLeftEncoderOffset = RobotName.select(1.734470, 0.608);
    public static final double kFrontRightEncoderOffset = RobotName.select(3.861472, 3.911);
    public static final double kRearLeftEncoderOffset = RobotName.select(4.897498, 3.870);
    public static final double kRearRightEncoderOffset = RobotName.select(2.850755, 0.706);

    public static final double kDriveWheelDiameterMeters = 0.2445;

    // Auton path finding controllers
    public static final PIDController kXController = new PIDController(0.100506, 0.0, 0.0);
    public static final PIDController kYController = new PIDController(0.1, 0.0, 0.0);

    // High profile constraints = pure P controller
    public static final ProfiledPIDController kThetaController =
        new ProfiledPIDController(
            9.0, 0.0, 0.80, new TrapezoidProfile.Constraints(1000.0, 100000.0));
  }

  public static final class Shooter {
    public static final class Accelerator {
      public static final boolean kIntert = true;
      public static final int kprimarySparkCanId = 13;
      public static final PIDGains kPidGains = new PIDGains(0.001, 0.0, 0.0);
      public static final DCMotor kSimMotor = DCMotor.getNEO(1);
      public static final double kGearRatio = 1.0;
      // Calculated accelerator intertia (from CAD) + NEO rotor inertia
      public static final double kInertia = (0.00042187) + (0.00003973);
      public static final SimpleMotorFeedforward kFeedforward =
          new SimpleMotorFeedforward(0.00, 0.00205);
      public static final int kFeederCanId = 14;
      public static final double kFeederInertia = (0.00003973);
      public static final double kFeederVoltage = 9.0;
      public static final boolean kFeederInvert = true;
    }

    public static final class Flywheel {

      public static final int kprimarySparkCanId = 12;
      public static final PIDGains kPidGains = new PIDGains(0.001, 0.0, 0.0);
      public static final DCMotor kSimMotor = DCMotor.getNEO(1);
      public static final double kGearRatio = 1.5;
      // Calculated flywheel intertia (from CAD) + NEO rotor inertia
      public static final double kInertia = (0.0030982) + (0.00003973);
      public static final SimpleMotorFeedforward kFeedforward =
          new SimpleMotorFeedforward(0.0, 12.0 / 3784.0);
      public static final double kAllowedVelocityError = 0.1;
    }

    public static final double kIdleRpm = 1500.0;
    public static final double kDefaultShotRpm = 3000.0;
  }

  public static final class Turret {
    public static final int kCanId = 25;
    public static final PIDGains kPositionPID = new PIDGains(0.05, 0.0, 0.0);
    public static final Constraints kProfileConstraints =
        new TrapezoidProfile.Constraints(30000.0, 60000.0);
    public static final double kForwardSoftLimit = 44.0;
    public static final double kReverseSoftLimit = -38.0;
    public static final double kStartingPosition = 0.0;
    public static final double kGearRatio = 76608.0 / 609.0; // (84 / 29) * (76 / 21) * (12 / 1);
    public static final double kInteria = 0.0 + (0.00003973);
    public static final double kS = 0.0; // 0.080665;
    public static final double kV = 0.0; // 0.057707;
    public static final Translation2d kTurretToChassis =
        new Translation2d(Units.inchesToMeters(-2.75), 0.0);
    public static final Translation2d kLimelightToTurret =
        new Translation2d(Units.inchesToMeters(-7.52), 0.0);
  }

  public static final class OIConstants {
    public static final int kDriverControllerPort = 0;
    public static final int kOperatorControllerPort = 1;
    public static final double kRightXDeadband = 0.1;
    public static final double kRightYDeadband = 0.1;
    public static final double kLeftXDeadband = 0.1;
    public static final double kLeftYDeadband = 0.1;

    // When either driver is ready to shoot, value is [0, 1]
    public static final double kShootRumbleStrength = 0.5;
  }

  public static final class Indexer {
    public static final int kFlappyRollerCanId = 22;
    public static final double kFlappyRollerVoltage = 5.0;
    public static final double kFlappyRollerSlowVoltage = 2.0;
    public static final boolean kFlappyRollerInvert = RobotName.select(true, false);
    public static final int kIndexerCanId = 21;
    public static final double kIndexerGearRatio = 5.321;

    // Threshold for a game piece to be present on the ball sensor in mm
    public static final double kIndexerSensorThreshold = 40.0;
    public static final double kIndexerReverseVoltage = -4.0;
    public static final double kIndexerForwardVoltage = 5.0; // 11.0;
    public static final double kIndexerShootVoltage = 11.0;
    public static final PIDGains kIndexerPositionGains = new PIDGains(0.1, 0.0, 0.0);
    public static final double kCurrentThreshold = 6.0;
    public static final double kCurrentHysterisis = 2.0;
    public static final int kFirstSensorDIO = 5;
    public static final int kSecondSensorDIO = 4;

    public static final double kIndexerAdjustRadians = Units.degreesToRadians(-600);
  }

  public static final class Hood {

    public static final float kForwardSoftLimit = 52;
    public static final float kReverseSoftLimit = 0;
    public static final double kGearRatio = 508.04;
    public static final int kCanId = 26;
    public static final PIDGains kPositionPID = new PIDGains(0.25, 0.0, 0.0);
    public static final double kStartingPosition = 0;
    public static final double kInteria = 0.0 + (0.00003973);
    public static final double kRezeroTimeoutSeconds = 5.0;
    public static final double kRezeroVoltage = -4.0;
    public static final double kRezeroBlankingTimeSeconds = 0.25;
    public static final double kRezeroCurrentThresholdAmps = 8.0;
    public static final double kRezeroVelocityThreshold = 5.0;
  }

  public static final class Intake {
    public static final int kCanId = 16;
    public static final int kFwdSolenoidChannel = 0;
    public static final int kRevSolenoidChannel = 1;
    public static final double kIntakeVoltage = 9.0;
    public static final boolean kSolenoidInvert = false;
  }

  public static final class Elevator {
    // Set up Variables
    public static final int kCanId = 31;
    public static final int kFollowerCanId = 10;
    public static final float kForwardSoftLimit = 167;
    public static final float kReverseSoftLimit = 0;

    // Calculations

    // Encoder
    public static final double kGearRatio = 0;

    // Feed Forward
    public static final double kS = 0.025; // units in volts
    public static final double kG = 6.05; // units in volts
    public static final double kV = 3.07; // volts * seconds / distance
    public static final double kA = 0.62; // volts * seconds^2 / distance
    public static final ElevatorFeedforward kElevatorFeedForward =
        new ElevatorFeedforward(kS, kG, kV, kA);

    // PID
    public static final double kMaxVelocity = 0.0;
    public static final double kMaxAcceleration = 0.0;
    public static final PIDGains kPositionPID = new PIDGains(0.2, 0.0, 0.0);
    public static final Constraints kProfileConstraints =
        new TrapezoidProfile.Constraints(kMaxVelocity, kMaxAcceleration);

    // Postitions
    public static final double kPositionTolerance = 0.25;
    public static final double kStartingPosition = 0; // all the way down
    public static final double kMidRungPosition = 166; // mid rung height
  }

  public static final class Sword {
    // Set up Variables
    public static final int kCanId = 31;
    public static final float kForwardSoftLimit = (float) 45.64;
    public static final float kReverseSoftLimit = 0;

    // Calculations

    // Encoder
    public static final double kGearRatio = 12 / 50;

    // Feed Forward
    public static final double kS = 0.025; // units in volts
    public static final double kG = 3.74; // units in volts
    public static final double kV = 3.07; // volts * seconds / distance
    public static final double kA = 0.38; // volts * seconds^2 / distance
    public static final ElevatorFeedforward kSwordFeedForward =
        new ElevatorFeedforward(kS, kG, kV, kA);

    // PID
    public static final double kMaxVelocity = 0.0;
    public static final double kMaxAcceleration = 0.0;
    public static final PIDGains kPositionPID = new PIDGains(0.2, 0.0, 0.0);
    public static final Constraints kProfileConstraints =
        new TrapezoidProfile.Constraints(kMaxVelocity, kMaxAcceleration);

    // Postitions
    public static final double kPositionTolerance = 0.25;
    public static final double kStartingPosition = 0; // all the way down
    public static final double kMidRungPosition = 45.5; // high rung height
  }

  public static final class Spears {
    public static final int kFwdSolenoidChannel = 2;
    public static final int kRevSolenoidChannel = 3;
    public static final boolean kSolenoidInvert = false;
  }

  public static final class Climber {
    public static final int kExtendCanId = 9;
    public static final int kExtendFollowerCanId = 31;
    public static final int kPivotFollowerCanId = 11;
    public static final int kRotateCanId = 10;
    public static final int kFwdSolenoidChannel = 2;
    public static final int kRevSolenoidChannel = 3;
    public static final Value kSolenoidEngagedValue = Value.kForward;

    public static final double kPivotGearReduction = 562.5;
    public static final float kPivotSoftLimitFwd = 28000.0f;
    public static final float kPivotSoftLimitRev = 0.0f;
    public static final int kPivotCurrentLimit = 22; // amps
    public static final double kPivotRampRate = 0.2; // time (sec) to go from 0% to 100% throttle
    public static final SimpleMotorFeedforward kPivotFeedforwardLoaded =
        new SimpleMotorFeedforward(0.0, 1 / 5880.0, 0.0);
    public static final TrapezoidProfile.Constraints kPivotConstraints =
        new TrapezoidProfile.Constraints(4000.0 / 60.0, 6000.0 / 60.0);
    public static final double kPivotPositionTolerance = 1.5;
    public static final PIDGains kPivotPositionPID = new PIDGains(0.2, 0.0, 0.0);

    public static final float kLinearSoftLimitFwd = 51.5f;
    public static final float kLinearSoftLimitRev = 0;
    public static final int kLinearCurrentLimitExtend = 35;
    public static final int kLinearCurrentLimitRetract = 80;
    public static final ElevatorFeedforward kLinearFeedforwardLoaded =
        new ElevatorFeedforward(0.025, -0.16, 1 / 5880.0, 0.0);
    public static final TrapezoidProfile.Constraints kLinearConstraints =
        new TrapezoidProfile.Constraints(3000.0 / 60.0, 6000.0 / 60.0);
    public static final double kLinearPositionTolerance = 1.0;
    public static final PIDGains kLinearPositionPID = new PIDGains(0.15, 0.0, 0.0);
    public static final int kLinearHookTriggerCurrent = 16;
    public static final double kLinearHookMotorPower = -0.2;

    public static final double kPresetLinearOut = 51.5;
    public static final double kPresetLinearSafe = 30.0;
    public static final double kPresetLinearDrop = 8.0;
    public static final double kPresetLinearIn = 0.0;

    public static final double kPresetPivotDrive = 50.0;
    public static final double kPresetPivotReady = 0.0;
    public static final double kPresetPivotHook = 25.0;
    public static final double kPresetPivotReach = 90.0;
    public static final double kPresetPivotHandoff = 68.0;
    public static final double kPresetPivotReachReverse = 85.0;
    public static final double kPresetPivotHookReverse = 25.0;
  }

  public static final class ShotPresets {
    public static double kDunkRPM = 1000;
    public static double kDunkHood = 25;
    public static double kShot1RPM = 2500;
    public static double kShot1Hood = 46;
    public static double kShot2RPM = 2700;
    public static double kShot2Hood = 48;
    public static double kShot3RPM = 2800;
    public static double kShot3Hood = 50;
  }

  public static final class Vision {
    public static final double kCameraAngleDegrees = 29;
    public static final double kTargetHeightMeters = Units.inchesToMeters(103);
    public static final double kCameraHeightMeters = Units.inchesToMeters(29.14 + 9.09);
    public static final double kTurretGain = 0.6;
    public static final int kTurretMovingAverageSampleLength = 2;
    public static final double kLimelightXOffset = 0;

    // spotless:off
    // Lookup tables are pairs of (distance meters, mechanism value)
    public static final LinearInterpolatedTable2d kHoodLookup =
        new LinearInterpolatedTable2d()
        .withPair(1.97+0.4, 39 - 23.8256 + 1)
        .withPair(2.46+0.4, 45 - 23.8256 + 1)
        .withPair(2.84+0.4, 48 - 23.8256 + 1)
        .withPair(3.20+0.4, 49 - 23.8256 + 4) //2
        .withPair(3.77+0.4, 53 - 23.8256 + 4) //2
        .withPair(4.24+0.4, 54 - 23.8256 + 3)
        .withPair(4.76+0.4, 57 - 23.8256 + 3)
        .withPair(5.61+0.4, 59 - 23.8256 + 3);

    public static final LinearInterpolatedTable2d kRpmLookup = 
        new LinearInterpolatedTable2d()
        .withPair(1.97+0.4, 2300+75) //everything was +125 at the end of states
        .withPair(2.46+0.4, 2350+75)
        .withPair(2.84+0.4, 2450+75)
        .withPair(3.20+0.4, 2500+75)
        .withPair(3.77+0.4, 2725+75)
        .withPair(4.24+0.4, 2800+75)
        .withPair(4.76+0.4, 2975+75)
        .withPair(5.61+0.4, 3150+75); //all 50s for red
    // spotless:on

    public static final double kTargetTimeoutSeconds = 5.0;
  }

  public static final class Robot {
    public static final double kMassLbs = 115.0;
    public static final double kMassKg = Units.lbsToKilograms(kMassLbs);
  }

  public static final class Field {
    public static final double kLength = Units.inchesToMeters(54 * 12);
    public static final double kMaxX = kLength;
    public static final double kWidth = Units.inchesToMeters(27 * 12);
    public static final double kMaxY = kWidth;

    // Distance from center of goal to vision tape
    public static final double kGoalRadius = Units.inchesToMeters(53.38 / 2);

    private static final Translation2d kGoalPosition =
        new Translation2d(Constants.Field.kMaxX / 2, Constants.Field.kMaxY / 2);
    public static final Pose2d kGoalPose = new Pose2d(kGoalPosition, new Rotation2d());
  }

  /**
   * Whether in competition mode or not. Competition mode has less telemetry (specific to tuning
   * controls), less error message, and less verbose logging. It may also have small changes to
   * calibration times e.g. gyro. Manually set this value before events.
   */
  public static final boolean kCompetitionMode = false;
}
