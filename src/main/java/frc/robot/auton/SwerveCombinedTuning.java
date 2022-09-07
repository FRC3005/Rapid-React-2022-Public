package frc.robot.auton;

import com.pathplanner.lib.PathPlannerTrajectory;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.lib.swerve.SwerveDrive;
import frc.robot.Constants;

public class SwerveCombinedTuning extends AutonCommandBase {
  public SwerveCombinedTuning(SwerveDrive swerve) {
    PathPlannerTrajectory trajectory = AutonUtils.loadTrajectory("CombinedTuning", 3.0, 5.0);
    Pose2d initialPose = new Pose2d(trajectory.getInitialPose().getTranslation(), new Rotation2d());

    // spotless:off
    addCommands(
        new InstantCommand(() -> swerve.resetOdometry(initialPose), swerve)
            .withName("Reset Odometry"),
        swerve.trajectoryFollowerCommand(
            trajectory,
            Constants.Drivetrain.kXController,
            Constants.Drivetrain.kYController,
            //new ProfiledPIDController(0.0, 0.0, 0.0, new TrapezoidProfile.Constraints(0.0, 0.0))),
            Constants.Drivetrain.kThetaController),
        new InstantCommand(() -> swerve.drive(0.0, 0.0, 0.0, false), swerve)
    );
    // spotless:on
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    super.initSendable(builder);
  }
}
