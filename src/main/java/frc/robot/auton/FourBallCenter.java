package frc.robot.auton;

import com.pathplanner.lib.PathPlannerTrajectory;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.VisionTrackingCommands;
import frc.robot.subsystems.Hood;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.subsystems.indexer.Indexer;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.turret.Turret;
import org.tinylog.Logger;

public class FourBallCenter extends AutonCommandBase {
  public FourBallCenter(
      DriveSubsystem drivebase,
      Shooter shooter,
      Indexer indexer,
      Intake intake,
      Hood hood,
      Turret turret,
      VisionTrackingCommands visionCommands) {

    PathPlannerTrajectory firstPath =
        AutonUtils.loadTrajectory("FourBallCenter_FirstBall", 2.0, 5.0);
    PathPlannerTrajectory secondPath =
        AutonUtils.loadTrajectory("FourBallCenter_SecondBall", 2.0, 5.0);
    PathPlannerTrajectory thirdPath = AutonUtils.loadTrajectory("AwayFromFeeder", 3.0, 6.0);

    double kIntakeTimeout = 1.5;
    double kShootTimeout = 0.8;
    Pose2d initialPose = AutonUtils.initialPose(firstPath);

    if (firstPath == null || secondPath == null || thirdPath == null) {
      Logger.tag("Four Ball Auton").error("Failed to load path");
      return;
    }

    /* Uses no trajectory, just super simple drive back for time, and shoot */
    // spotless:off
    addCommandsWithLog("Four Ball",
      // House keeping commands for pose and gyro
      new InstantCommand(() -> drivebase.setHeading(initialPose.getRotation().getDegrees())),
      new InstantCommand(() -> drivebase.resetOdometry(initialPose), drivebase).withName("Reset Odometry"),

      // Enable tracking, turn on shooter and hood
      visionCommands.enableTracking(),
      hood.enableCommand(),

      // Drive back while intaking
      new ParallelCommandGroup(
          indexer.fastIntakeCommand().raceWith(
            intake.intakeOutCommand(),
            shooter.spinUpCommand()
          ),
          new WaitCommand(0.4).andThen(
          drivebase.trajectoryFollowerCommand(firstPath)
              .withTimeout(1.8))
              .andThen(() -> drivebase.stop(), drivebase).withName("Stop")
      ).withTimeout(kIntakeTimeout),

      new InstantCommand(() -> Logger.tag("Four Ball").warn("Finished first path")),
      //visionCommands.waitOnTarget(2.0),

      // Stop and bring the intake in before shooting to be more stable
      new InstantCommand(() -> drivebase.stop(), drivebase),
      new WaitCommand(0.25),
      shooter.shootCommand().withTimeout(kShootTimeout),
      shooter.postShootCommand(),
      new InstantCommand((() -> turret.setDegrees(0))).withName("Set turret to 0"),

      new InstantCommand(() -> Logger.tag("Four Ball").warn("Starting second path")),
      // Drive back while intaking
      new ParallelCommandGroup(
          indexer.fastIntakeCommand().raceWith(
            intake.intakeOutCommand()
          ),
          drivebase.trajectoryFollowerCommand(secondPath)
              .andThen(() -> drivebase.stop(), drivebase).withName("Stop")
      ).withTimeout(3.0),

      // Wait for both game pieces (maybe lol)
      new WaitCommand(1.4),
      intake.intakeInCommand(),
      drivebase.trajectoryFollowerCommand(thirdPath)
          .andThen(() -> drivebase.stop(), drivebase).withName("Stop"),

      new InstantCommand(() -> drivebase.stop(), drivebase),
      new WaitCommand(0.8),
      shooter.shootCommand().withTimeout(kShootTimeout),
      shooter.postShootCommand(),

      new InstantCommand(() -> drivebase.stop(), drivebase),
      turret.setDegreesCommand(0.0),
      intake.intakeInCommand()
    );
    // spotless:on
  }
}
