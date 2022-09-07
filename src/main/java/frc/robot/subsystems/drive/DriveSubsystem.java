package frc.robot.subsystems.drive;

import com.pathplanner.lib.PathPlannerTrajectory;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj2.command.Command;
import frc.lib.swerve.SwerveDrive;
import frc.lib.swerve.SwerveModule;
import frc.lib.vendor.sensor.SendableGyro;
import frc.lib.vendor.sensor.ThroughBoreEncoder;
import frc.robot.Constants;

public class DriveSubsystem extends SwerveDrive {
  public Command trajectoryFollowerCommand(PathPlannerTrajectory trajectory) {
    Constants.Drivetrain.kThetaController.enableContinuousInput(-Math.PI, Math.PI);
    return trajectoryFollowerCommand(
        trajectory,
        Constants.Drivetrain.kXController,
        Constants.Drivetrain.kYController,
        Constants.Drivetrain.kThetaController);
  }

  static final SwerveModule frontLeft =
      new NickSwerveModule(
          Constants.Drivetrain.kFrontLeftDriveCanId,
          Constants.Drivetrain.kFrontLeftTurningCanId,
          new ThroughBoreEncoder(
              Constants.Drivetrain.kFrontLeftAbsoluteEncoderPort,
              Constants.Drivetrain.kFrontLeftEncoderOffset,
              2 * Math.PI,
              true));

  static final SwerveModule frontRight =
      new NickSwerveModule(
          Constants.Drivetrain.kFrontRightDriveCanId,
          Constants.Drivetrain.kFrontRightTurningCanId,
          new ThroughBoreEncoder(
              Constants.Drivetrain.kFrontRightAbsoluteEncoderPort,
              Constants.Drivetrain.kFrontRightEncoderOffset,
              2 * Math.PI,
              true));

  static final SwerveModule rearLeft =
      new NickSwerveModule(
          Constants.Drivetrain.kRearLeftDriveCanId,
          Constants.Drivetrain.kRearLeftTurningCanId,
          new ThroughBoreEncoder(
              Constants.Drivetrain.kRearLeftAbsoluteEncoderPort,
              Constants.Drivetrain.kRearLeftEncoderOffset,
              2 * Math.PI,
              true));

  static final SwerveModule rearRight =
      new NickSwerveModule(
          Constants.Drivetrain.kRearRightDriveCanId,
          Constants.Drivetrain.kRearRightTurningCanId,
          new ThroughBoreEncoder(
              Constants.Drivetrain.kRearRightAbsoluteEncoderPort,
              Constants.Drivetrain.kRearRightEncoderOffset,
              2 * Math.PI,
              true));

  public DriveSubsystem(SendableGyro gyro) {
    super(
        frontLeft,
        frontRight,
        rearLeft,
        rearRight,
        Constants.Drivetrain.kDriveKinematics,
        gyro,
        Constants.Drivetrain.kMaxDriveSpeed);
    // Logger.tag("Swerve Drive").warn("Reset calibration time back to longer for comp");
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    super.initSendable(builder);
    addChild("X Controller", Constants.Drivetrain.kXController);
    addChild("Y Controller", Constants.Drivetrain.kYController);
    addChild("Theta Controller", Constants.Drivetrain.kThetaController);
  }
}
