package frc.robot.auton;

import com.pathplanner.lib.PathPlannerTrajectory;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import frc.lib.swerve.SwerveDrive;
import frc.robot.Constants;

public class SwerveDriveBackTuning extends AutonCommandBase {
  public SwerveDriveBackTuning(SwerveDrive swerve) {
    PathPlannerTrajectory trajectory = AutonUtils.loadTrajectory("Drive2M", 2.0, 6.0, true);
    // spotless:off
    addCommands(
        new InstantCommand(() -> swerve.resetOdometry(trajectory.getInitialPose()), swerve)
            .withName("Reset Odometry"),
        swerve.trajectoryFollowerCommand(
            trajectory,
            Constants.Drivetrain.kXController,
            Constants.Drivetrain.kYController,
            new ProfiledPIDController(0.0, 0.0, 0.0, new TrapezoidProfile.Constraints(0.0, 0.0))),
        new InstantCommand(() -> swerve.drive(0.0, 0.0, 0.0, false), swerve)
    );
    // spotless:on
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    super.initSendable(builder);
  }
}
