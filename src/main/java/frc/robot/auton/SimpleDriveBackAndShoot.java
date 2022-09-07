package frc.robot.auton;

import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.auton.locations.Location;
import frc.robot.commands.VisionTrackingCommands;
import frc.robot.subsystems.Hood;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.climber.Climber;
import frc.robot.subsystems.drive.DriveSubsystem;
import frc.robot.subsystems.indexer.Indexer;
import frc.robot.subsystems.shooter.Shooter;
import frc.robot.subsystems.turret.Turret;
import org.tinylog.Logger;

public class SimpleDriveBackAndShoot extends AutonCommandBase {
  public SimpleDriveBackAndShoot(
      DriveSubsystem drivebase,
      Shooter shooter,
      Indexer indexer,
      Intake intake,
      Hood hood,
      Turret turret,
      Climber climber,
      VisionTrackingCommands visionCommands,
      Location startLocation) {

    super(startLocation.getName().replace("Start", "") + "Drive Back SIMPLE");

    double kDriveTime = 1.7;
    double kIntakeTimeout = 3.0;

    // Percent
    double kDriveSpeed = 0.15;

    /* Uses no trajectory, just super simple drive back for time, and shoot */
    // spotless:off
    addCommandsWithLog("Simple Drive Back and Shoot " + startLocation.getName(),
        // House keeping commands for pose and gyro
        new InstantCommand(() -> drivebase.setHeading(startLocation.get().getRotation().getDegrees())).withName("Set Heading"),
        new InstantCommand(() -> drivebase.resetOdometry(startLocation.get())).withName("Reset Odometry"),

        // Enable tracking, turn on shooter and hood
        visionCommands.enableTracking(),
        hood.enableCommand(),

        // Drive back while intaking
        new ParallelCommandGroup(
            indexer.fastIntakeCommand().raceWith(
              intake.intakeOutCommand(),
              shooter.spinUpCommand()
            ).withName("Intake Out Spinup"),
            new RunCommand(() -> drivebase.drive(kDriveSpeed, 0.0, 0.0, false), drivebase)
                .withTimeout(kDriveTime)
                .withName("Drive Back Time")
                .andThen(() -> drivebase.stop()).withName("Drive x Speed for Y Time")
        ).withTimeout(kIntakeTimeout)
        .withName("Drive back while intaking"),

        new InstantCommand(() -> Logger.tag("Simple Drive Back and Shoot").info("Finished drive back")),
        visionCommands.waitOnTarget(2.0),

        // Stop and bring the intake in before shooting to be more stable
        new InstantCommand(() -> drivebase.stop()).withName("Stop Drive"),
        intake.intakeInCommand(),
        new WaitCommand(1.0).withName("Wait for intake"),
        shooter.shootCommand().withTimeout(4.0),

        // Disable tracking and reset hood and turret
        visionCommands.disableTracking(),
        turret.setDegreesCommand(0.0),
        hood.disableCommand()
    );
    // spotless:on
  }
}
