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

public class TwoPlusTwoLeft extends AutonCommandBase {
  public TwoPlusTwoLeft(
      DriveSubsystem drivebase,
      Shooter shooter,
      Indexer indexer,
      Intake intake,
      Hood hood,
      Turret turret,
      VisionTrackingCommands visionCommands) {

    PathPlannerTrajectory firstPath =
        AutonUtils.loadTrajectory("TwoPlusTwo_Left_FirstBall", 2.0, 5.0);
    PathPlannerTrajectory secondPath = AutonUtils.loadTrajectory("TwoPlusTwo_Left_Steal", 3.0, 5.0); //TwoPlusTwo_Left_Steal
    PathPlannerTrajectory thirdPath = AutonUtils.loadTrajectory("TwoPlusTwo_Left_End", 3.0, 5.0);

    double kIntakeTimeout = 1.3;
    double kShootTimeout = 0.8;
    double kPoopTime = 2.0;
    Pose2d initialPose = AutonUtils.initialPose(firstPath);

    if (firstPath == null || secondPath == null || thirdPath == null) {
      Logger.tag("2+2 Left").error("Failed to load path");
      return;
    }

    /* Uses no trajectory, just super simple drive back for time, and shoot */
    // spotless:off
    addCommandsWithLog("2+2 Left",
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
              .withTimeout(1.7))
              .andThen(() -> drivebase.stop(), drivebase).withName("Stop")
      ).withTimeout(kIntakeTimeout),

      new InstantCommand(() -> Logger.tag("2+2 Left").warn("Finished first path")),
      //visionCommands.waitOnTarget(2.0),

      // Stop and shoot
      new InstantCommand(() -> drivebase.stop(), drivebase),
      new WaitCommand(0.25),
      shooter.shootCommand().withTimeout(kShootTimeout),
      shooter.postShootCommand(),
      new InstantCommand((() -> turret.setDegrees(0))).withName("Set turret to 0"),

      new InstantCommand(() -> Logger.tag("2+2 Left").info("Starting second path")),
      // Intake again while driving next path
      new ParallelCommandGroup(
          indexer.fastIntakeCommand().raceWith(
            intake.intakeOutCommand()
          ),
          drivebase.trajectoryFollowerCommand(secondPath)
              .andThen(() -> drivebase.stop())
      ).withTimeout(4.5),
      new InstantCommand(() -> Logger.tag("2+2 Left").warn("Finished second path")),

      new InstantCommand(() -> drivebase.stop(), drivebase),
      
      // Poop out the cargo
      intake.intakeOutReversedCommand().alongWith(indexer.reverseIndexerCommand()).withTimeout(kPoopTime),
      
      // Stop everything now (timeout since these are run commands)
      indexer.stopCommand().withTimeout(0.05),
      intake.intakeInCommand().withTimeout(0.05),

      drivebase.trajectoryFollowerCommand(thirdPath),
      new InstantCommand(() -> drivebase.stop(), drivebase)
    );
    // spotless:on
  }
}
