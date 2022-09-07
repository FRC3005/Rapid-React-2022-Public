package frc.robot.subsystems.robotstate;

import edu.wpi.first.math.Pair;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.FieldObject2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.lib.swerve.SwerveDrive;
import frc.lib.vendor.sensor.Limelight;
import frc.lib.vendor.sensor.SendableGyro;
import frc.lib.wpi.SwerveDrivePoseEstimator;
import frc.robot.Constants;
import java.util.function.DoubleSupplier;
import org.tinylog.Logger;

public class RobotState extends SubsystemBase {

  private final SwerveDrive m_swerve;
  private final Limelight m_limelight;
  private final DoubleSupplier m_turretDegreesSupplier;
  private final SendableGyro m_gyro;
  private Double m_lastVisionUpdate = Double.valueOf(0.0);
  private Pose2d m_lastVisionPose = new Pose2d();
  private boolean m_resetOnNextVisionUpdate = false;

  // Telemetry data
  private final Field2d m_field = new Field2d();
  private final FieldObject2d m_visionEstimate = m_field.getObject("VisionEst");
  private final FieldObject2d m_futurePoseEstimate = m_field.getObject("FutureEst");

  private final SwerveDrivePoseEstimator m_poseEstimator =
      new SwerveDrivePoseEstimator(
          new Rotation2d(),
          new Pose2d(),
          Constants.Drivetrain.kDriveKinematics,
          VecBuilder.fill(0.05, 0.05, Units.degreesToRadians(5)),
          VecBuilder.fill(Units.degreesToRadians(0.01)),
          VecBuilder.fill(0.3, 0.3, Units.degreesToRadians(20)));

  public RobotState(
      SwerveDrive swerve,
      Limelight limelight,
      SendableGyro gyro,
      DoubleSupplier turretDegreesSupplier) {
    m_swerve = swerve;
    m_turretDegreesSupplier = turretDegreesSupplier;
    m_limelight = limelight;
    m_gyro = gyro;
    SmartDashboard.putData(m_field);
  }

  /**
   * Get the pose of the robot using fused vision and wheel odometry measurements.
   *
   * @return pose of the robot based on vision and odometry measurements.
   */
  public Pose2d getPoseEstimate() {
    return m_poseEstimator.getEstimatedPosition();
  }

  /**
   * Get the pose of the robot some time in the future.
   *
   * @param lookaheadTime time in seconds to get the future pose.
   * @return pose of the robot relative to the field some time in the future.
   */
  public Pose2d getFuturePoseEstimate(double lookaheadTime) {
    Pose2d result =
        m_poseEstimator.getEstimatedPosition().exp(m_swerve.getPredictedMotion(lookaheadTime));
    m_futurePoseEstimate.setPose(result);
    return result;
  }

  /** Reset the robot pose estimation to the exact pose given by the next vision update. */
  public void resetPoseEstimateOnVisionUpdate() {
    m_resetOnNextVisionUpdate = true;
  }

  /**
   * Reset the pose estimation to a fixed pose.
   *
   * @param resetPose pose to set the estimator to.
   */
  public void resetPoseEstimate(Pose2d resetPose) {
    m_poseEstimator.resetPosition(resetPose, m_gyro.getRotation2d());
  }

  /**
   * Get the transform of the chassis from the turret angle
   *
   * @return Transform2d of turret from chassis
   */
  public Transform2d getTurretToChassis() {
    return new Transform2d(
        Constants.Turret.kTurretToChassis,
        Rotation2d.fromDegrees(m_turretDegreesSupplier.getAsDouble()));
  }

  /**
   * Get the pose of the turret from the chassis pose.
   *
   * @param chassisPose field relative robot pose
   * @return field relative turret pose.
   */
  public Pose2d getChassisToTurret(Pose2d chassisPose) {
    return chassisPose.transformBy(getTurretToChassis().inverse());
  }

  /**
   * Get the transform between any field relative pose and the center of the goal.
   *
   * @param pose field relative pose
   * @return transform between pose and goal
   */
  public static Transform2d getPoseToGoal(Pose2d pose) {
    return new Transform2d(pose, Constants.Field.kGoalPose);
  }

  /**
   * Get the transform from the limelight to the turret
   *
   * @return
   */
  public Transform2d getLimelightToTurret() {
    return new Transform2d(Constants.Turret.kLimelightToTurret, new Rotation2d());
  }

  /**
   * Return the translation from the limelight to the center of the goal. This adds the extra
   * distance from the tape marker to the center of the goal.
   *
   * @param targetTranslation output of limelight getTargetTranslation()
   * @return
   */
  public static Translation2d centerOfGoalFromVisionTranslation(Translation2d targetTranslation) {
    double angle = Math.atan2(targetTranslation.getY(), targetTranslation.getX());
    return targetTranslation.plus(
        new Translation2d(Constants.Field.kGoalRadius, new Rotation2d(angle)));
  }

  /**
   * Get the timestamp of the last successful vision update.
   *
   * @return last vision update in seconds.
   */
  public double getLastVisionUpdate() {
    return m_lastVisionUpdate;
  }

  /**
   * Get last image pose from the last captured image.
   *
   * @return last pose reported by the camera.
   */
  public Pose2d getLastVisionToChassis() {
    return m_lastVisionPose;
  }

  /**
   * Return the pose of the chassis relative to the field from the vision target.
   *
   * @return Pair of Pose2d, Double where Pose2d is the robot pose relative to the field, and Double
   *     is the timestamp of the measurement. Returns null if no new update.
   */
  private Pair<Pose2d, Double> getChassisFromVision() {
    double visionTimestamp = m_limelight.getLastTimestamp() - m_limelight.getLatency();

    if (visionTimestamp > m_lastVisionUpdate && m_limelight.isValidTarget()) {
      SmartDashboard.putNumber("Vision Pose Update", visionTimestamp);
      m_lastVisionUpdate = visionTimestamp;

      // See scripts/pose_estimation for breakdown of the below
      Translation2d cameraToTarget =
          centerOfGoalFromVisionTranslation(m_limelight.getTargetTranslation());
      Pose2d fieldToLimelight =
          Constants.Field.kGoalPose.transformBy(
              new Transform2d(
                      cameraToTarget,
                      Rotation2d.fromDegrees(
                          -(m_turretDegreesSupplier.getAsDouble()
                              + m_gyro.getRotation2d().getDegrees())))
                  .inverse());
      Pose2d fieldToTurret = fieldToLimelight.transformBy(getLimelightToTurret());
      Pose2d fieldToRobot = fieldToTurret.transformBy(getTurretToChassis());

      return Pair.of(fieldToRobot, Double.valueOf(visionTimestamp));
    } else {
      return null;
    }
  }

  /**
   * Get the absolute pose of the target on the field based on vision. This will return a pose that
   * is not the same as Constants.Vision.kGoalPose. Due to the drift of the robot odometry we can
   * use this position to target no matter where the robot has drifted to. We rely on the fact that
   * the odometry drift is over time, so the past second or two should be fairly accurate.
   *
   * <p>Used specifically for targeting, not for position correction on the field.
   *
   * @return pair of Pose2d Double where Pose2d is the measured position of the goal from vision,
   *     and double is a timestamp.
   */
  private Pair<Pose2d, Double> getVisionBasedFieldToTarget() {
    return null;
  }

  @Override
  public void periodic() {
    var visionUpdate = getChassisFromVision();

    if (visionUpdate != null && false) {
      if (m_resetOnNextVisionUpdate) {
        Logger.tag("Robot State")
            .info(
                "Resetting pose estimator from vision update, pose: {}, timestamp: {}",
                visionUpdate.getFirst(),
                visionUpdate.getSecond());
        resetPoseEstimate(visionUpdate.getFirst());
        m_resetOnNextVisionUpdate = false;
      } else {
        m_poseEstimator.addVisionMeasurement(visionUpdate.getFirst(), visionUpdate.getSecond());
      }

      m_lastVisionPose = visionUpdate.getFirst();
      m_visionEstimate.setPose(m_lastVisionPose);
      SmartDashboard.putNumber(
          "Vision Pose Est/Theta (degrees)", visionUpdate.getFirst().getRotation().getDegrees());
      SmartDashboard.putNumber(
          "Vision Pose Est/X (Meters)", visionUpdate.getFirst().getTranslation().getX());
      SmartDashboard.putNumber(
          "Vision Pose Est/Y (Meters)", visionUpdate.getFirst().getTranslation().getY());
    }
    // TODO: Get timestamps for swerve odometry or otherwise correct for it
    // m_field.setRobotPose(
    //    m_poseEstimator.update(m_gyro.getRotation2d(), m_swerve.getModuleStates()));
    SmartDashboard.putNumber("Robot Pose Est/X (Meters)", m_field.getRobotPose().getX());
    SmartDashboard.putNumber("Robot Pose Est/Y (Meters)", m_field.getRobotPose().getY());
    SmartDashboard.putNumber(
        "Robot Pose Est/Theta (degrees)", m_field.getRobotPose().getRotation().getDegrees());
    SmartDashboard.putNumber(
        "Robot Pose Est/Future X (Meters)", m_futurePoseEstimate.getPose().getX());
    SmartDashboard.putNumber(
        "Robot Pose Est/Future Y (Meters)", m_futurePoseEstimate.getPose().getY());
    SmartDashboard.putNumber(
        "Robot Pose Est/Future Theta (degrees)",
        m_futurePoseEstimate.getPose().getRotation().getDegrees());
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    super.initSendable(builder);
    builder.addDoubleProperty(
        "Target Distance",
        () -> centerOfGoalFromVisionTranslation(m_limelight.getTargetTranslation()).getNorm(),
        null);
  }

  public void testModePeriodic() {
    periodic();
  }
}
