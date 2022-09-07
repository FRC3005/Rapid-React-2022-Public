package frc.robot.auton;

import com.pathplanner.lib.PathPlannerTrajectory;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import frc.robot.subsystems.Hood;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.subsystems.indexer.Indexer;
import frc.robot.subsystems.shooter.Shooter;

public class DriveBackAndShootRight extends AutonCommandBase {
  public DriveBackAndShootRight(
      DriveSubsystem drivebase, Shooter shooter, Indexer indexer, Intake intake, Hood hood) {
    PathPlannerTrajectory trajectory = AutonUtils.loadTrajectory("FirstBallRight", 2.0, 3.0);

    if (trajectory == null) {
      // TODO: Revert to some default
      return;
    }

    addCommands(
        new InstantCommand(() -> drivebase.resetOdometry(trajectory.getInitialPose()), drivebase)
            .withName("Reset Odometry"),
        new ParallelCommandGroup(
            intake.intakeOutCommand(),
            indexer.intakeCommand(),
            drivebase
                .trajectoryFollowerCommand(trajectory)
                .andThen(new InstantCommand(() -> drivebase.drive(0, 0, 0, false), drivebase))),
        shooter.shootCommand());
  }
}
