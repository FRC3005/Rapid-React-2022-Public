package frc.robot.subsystems.drive;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel.MotorType;
import com.revrobotics.CANSparkMaxLowLevel.PeriodicFrame;
import com.revrobotics.CANSparkMaxSim1;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.util.sendable.SendableBuilder;
import edu.wpi.first.wpilibj.RobotBase;
import edu.wpi.first.wpilibj.RobotController;
import frc.lib.controller.Controller;
import frc.lib.electromechanical.Encoder;
import frc.lib.sim.ThroughBoreEncoderSim;
import frc.lib.sim.wpiClasses.SwerveModuleSim;
import frc.lib.swerve.SwerveModule;
import frc.lib.util.SendableHelper;
import frc.lib.vendor.motorcontroller.SparkMax;
import frc.lib.vendor.motorcontroller.SparkMaxUtils;
import frc.lib.vendor.sensor.ThroughBoreEncoder;
import frc.robot.Constants;
import org.tinylog.Logger;

public class NickSwerveModule implements SwerveModule {
  private final SparkMax m_driveMotor;
  private final SparkMax m_turningMotor;
  private final ProfiledPIDController m_turningController;
  private final Controller m_driveController;

  private final Encoder m_turningEncoder;
  private final ThroughBoreEncoder m_turningAbsoluteEncoder;
  private final Encoder m_driveEncoder;

  private final SimpleMotorFeedforward m_driveFeedforward;
  private final SimpleMotorFeedforward m_turningFeedforward;

  private boolean m_testEnable = false;

  private SwerveModuleState m_desiredState = new SwerveModuleState(0.0, new Rotation2d());

  // Simulation Functions
  private final CANSparkMaxSim1 m_turningMotorSim;
  private final CANSparkMaxSim1 m_driveMotorSim;
  private final ThroughBoreEncoderSim m_absoluteEncoderSim;
  private final SwerveModuleSim m_sim =
      new SwerveModuleSim(
          DCMotor.getNeo550(1),
          DCMotor.getNEO(1),
          Constants.Drivetrain.kDriveWheelDiameterMeters / 2.0,
          Constants.Drivetrain.kTurningModuleGearRatio,
          Constants.Drivetrain.kDriveMotorReduction,
          Math.PI * 2, // Turning motor encoder ratio (TODO: we spit out radians, does this?)
          1.0,
          1.1, // TODO: How to quantify this?
          0.8, // TODO: How to quantify this?
          Constants.Robot.kMassKg,
          0.01); // Turning intertia TODO: How to quantify this?

  private static Boolean driveMotorConfig(CANSparkMax sparkMax, Boolean isInit) {
    int errors = 0;
    RelativeEncoder enc = sparkMax.getEncoder();

    // Convert 'rotations' to 'meters'
    // enc.setPositionConversionFactor(Constants.Drivetrain.kDriveEncoderPositionFactor);
    errors +=
        SparkMaxUtils.check(
            enc.setPositionConversionFactor(Constants.Drivetrain.kDriveEncoderPositionFactor));

    // Convert 'RPM' to 'meters per second'
    errors +=
        SparkMaxUtils.check(
            enc.setVelocityConversionFactor(Constants.Drivetrain.kDriveEncoderVelocityFactor));

    // Set inversion
    sparkMax.setInverted(Constants.Drivetrain.kDriveMotorInvert);
    sparkMax.setInverted(Constants.Drivetrain.kDriveMotorInvert);

    errors += SparkMaxUtils.check(sparkMax.getPIDController().setOutputRange(-1, 1));
    errors += SparkMaxUtils.check(sparkMax.setIdleMode(IdleMode.kCoast));
    errors +=
        SparkMaxUtils.check(
            sparkMax.setSmartCurrentLimit(Constants.Drivetrain.kDriveMotorCurrentLimit));

    sparkMax.setPeriodicFramePeriod(PeriodicFrame.kStatus0, 100);
    sparkMax.setPeriodicFramePeriod(PeriodicFrame.kStatus1, 15);
    sparkMax.setPeriodicFramePeriod(PeriodicFrame.kStatus2, 15);
    sparkMax.setPeriodicFramePeriod(PeriodicFrame.kStatus3, 200);

    return errors == 0;
  }

  private static Boolean turningMotorConfig(CANSparkMax sparkMax, Boolean isInit) {
    int errors = 0;
    RelativeEncoder enc = sparkMax.getEncoder();

    // Convert 'rotations' to 'meters'
    errors +=
        SparkMaxUtils.check(
            enc.setPositionConversionFactor(Constants.Drivetrain.kTurningEncoderPositionFactor));

    // Convert 'RPM' to 'meters per second'
    errors +=
        SparkMaxUtils.check(
            enc.setVelocityConversionFactor(Constants.Drivetrain.kTurningEncoderVelocityFactor));

    // Set inversion
    sparkMax.setInverted(Constants.Drivetrain.kTurningMotorInvert);
    sparkMax.setInverted(Constants.Drivetrain.kTurningMotorInvert);

    errors += SparkMaxUtils.check(sparkMax.getPIDController().setOutputRange(-1, 1));
    errors += SparkMaxUtils.check(sparkMax.setIdleMode(IdleMode.kBrake));
    errors += SparkMaxUtils.check(SparkMaxUtils.setDefaultsForNeo500(sparkMax));
    // sparkMax.setSmartCurrentLimit(Constants.Drivetrain.kTurningMotorCurrentLimit);
    // sparkMax.getEncoder().setPosition(0.0);

    sparkMax.setPeriodicFramePeriod(PeriodicFrame.kStatus0, 100);
    sparkMax.setPeriodicFramePeriod(PeriodicFrame.kStatus1, 15);
    sparkMax.setPeriodicFramePeriod(PeriodicFrame.kStatus2, 15);
    sparkMax.setPeriodicFramePeriod(PeriodicFrame.kStatus3, 200);

    return errors == 0;
  }

  public NickSwerveModule(int driveCanId, int turningCanId, ThroughBoreEncoder turningEncoder) {
    m_driveMotor =
        new SparkMax(driveCanId, MotorType.kBrushless)
            .withInitializer(NickSwerveModule::driveMotorConfig);
    m_turningMotor =
        new SparkMax(turningCanId, MotorType.kBrushless)
            .withInitializer(NickSwerveModule::turningMotorConfig);

    m_turningEncoder = m_turningMotor.builtinEncoder();
    m_driveEncoder = m_driveMotor.builtinEncoder();
    m_driveEncoder.setPosition(0.0);
    m_turningAbsoluteEncoder = turningEncoder;

    m_driveFeedforward = Constants.Drivetrain.kDriveFeedforward;
    m_turningFeedforward = Constants.Drivetrain.kTurningFeedforward;

    m_driveController = m_driveMotor.velocityController(Constants.Drivetrain.kDriveMotorPIDGains);

    m_turningController =
        new ProfiledPIDController(
            Constants.Drivetrain.kTurningMotorPIDGains.P,
            Constants.Drivetrain.kTurningMotorPIDGains.I,
            Constants.Drivetrain.kTurningMotorPIDGains.D,
            Constants.Drivetrain.kTurningConstraints);

    m_turningController.enableContinuousInput(-Math.PI, Math.PI);

    // Simulation init
    m_turningMotorSim = new CANSparkMaxSim1(m_turningMotor);
    m_driveMotorSim = new CANSparkMaxSim1(m_driveMotor);
    m_absoluteEncoderSim = new ThroughBoreEncoderSim(m_turningAbsoluteEncoder);

    // Set the position to the absolute values
    resetEncoders();

    // Reset the angle to current angle
    m_desiredState.angle = new Rotation2d(m_turningAbsoluteEncoder.getPosition());
  }

  @Override
  public void initSendable(SendableBuilder builder) {
    builder.setSmartDashboardType("SwerveModule");
    builder.setActuator(true);
    SendableHelper.addChild(builder, this, m_driveController, "DriveMotor");
    SendableHelper.addChild(builder, this, m_turningController, "TurningMotor");
    SendableHelper.addChild(builder, this, m_turningEncoder, "TurningEncoder");
    SendableHelper.addChild(builder, this, m_driveEncoder, "DriveEncoder");
    if (m_turningAbsoluteEncoder != null) {
      SendableHelper.addChild(builder, this, m_turningAbsoluteEncoder, "TurningAbsolute");
    }
    SendableHelper.addChild(builder, this, m_turningEncoder, "TurningEncoder");
    builder.addDoubleProperty(
        "Velocity Setpoint",
        () -> m_desiredState.speedMetersPerSecond,
        val -> setDesiredState(new SwerveModuleState(val, m_desiredState.angle)));
    builder.addDoubleProperty(
        "Turning Setpoint",
        () -> m_desiredState.angle.getRadians(),
        (val) ->
            setDesiredState(
                new SwerveModuleState(m_desiredState.speedMetersPerSecond, new Rotation2d(val))));
    builder.addBooleanProperty("Testmode Enable", () -> m_testEnable, (val) -> m_testEnable = val);
    builder.addDoubleProperty("Drive Current", () -> m_driveMotor.getOutputCurrent(), null);
    builder.addDoubleProperty("Turning Current", () -> m_turningMotor.getOutputCurrent(), null);
  }

  @Override
  public SwerveModuleState getState() {
    return new SwerveModuleState(
        m_driveEncoder.getVelocity(), new Rotation2d(m_turningAbsoluteEncoder.getPosition()));
  }

  @Override
  public void setDesiredState(SwerveModuleState desiredState) {
    // Module range must be [0, PI)
    // desiredState.angle = new Rotation2d(MathUtil.inputModulus(desiredState.angle.getRadians(),
    // 0.0, 2.0 * Math.PI));
    desiredState.angle = new Rotation2d(desiredState.angle.getRadians() % (2.0 * Math.PI));
    // Optimize the reference state to avoid spinning further than 90 degrees
    SwerveModuleState state =
        SwerveModuleState.optimize(
            desiredState, new Rotation2d(m_turningAbsoluteEncoder.getPosition()));

    // Module range must be [0, PI)
    desiredState.angle = new Rotation2d(desiredState.angle.getRadians() % (2.0 * Math.PI));

    m_desiredState = state;
  }

  @Override
  public SwerveModuleState getDesiredState() {
    return m_desiredState;
  }

  @Override
  public void periodic() {
    // Calculate the turning motor output from the turning PID controller.
    m_driveController.setReference(
        m_desiredState.speedMetersPerSecond,
        m_driveEncoder.getVelocity(),
        m_driveFeedforward.calculate(m_desiredState.speedMetersPerSecond));
    m_turningController.setGoal(m_desiredState.angle.getRadians());
    double demand = m_turningController.calculate(m_turningAbsoluteEncoder.getPosition());
    demand += m_turningFeedforward.calculate(m_turningController.getSetpoint().velocity);
    m_turningMotor.setVoltage(demand);
  }

  /** Run the controllers */
  @Override
  public void testPeriodic() {
    if (RobotBase.isSimulation()) {
      simulationPeriodic();
    }

    if (!m_testEnable) {
      m_turningMotor.setVoltage(0.0);
      return;
    }

    m_driveController.setReference(
        m_desiredState.speedMetersPerSecond,
        m_driveEncoder.getVelocity(),
        m_driveFeedforward.calculate(m_desiredState.speedMetersPerSecond));
    m_turningController.setGoal(m_desiredState.angle.getRadians());

    double demand = m_turningController.calculate(m_turningAbsoluteEncoder.getPosition());
    demand += m_turningFeedforward.calculate(m_turningController.getSetpoint().velocity);
    m_turningMotor.setVoltage(demand);
  }

  @Override
  public void simulationPeriodic() {
    m_turningMotorSim.iterateByPosition(
        m_sim.getAzimuthEncoderPositionRev(), RobotController.getBatteryVoltage(), 0.02);

    m_driveMotorSim.iterateByPosition(
        m_sim.getWheelEncoderPositionRev(), RobotController.getBatteryVoltage(), 0.02);

    // This is separate as it leaves room to adjust battery voltage each call based on current.
    double vbus = RobotController.getBatteryVoltage();
    m_sim.setInputVoltages(
        vbus * m_driveMotorSim.getAppliedOutput(), vbus * m_turningMotorSim.getAppliedOutput());

    m_absoluteEncoderSim.setAbsolutePosition(m_sim.getAzimuthEncoderPositionRev());
    m_sim.update(0.02);
  }

  @Override
  public void resetEncoders() {
    m_driveEncoder.setPosition(0);
    if (m_turningAbsoluteEncoder != null) {
      m_turningEncoder.setPosition(m_turningAbsoluteEncoder.getPosition());
    } else {
      Logger.tag("Swerve Module").warn("Unable to reset, no sensor to reset on");
    }
  }

  @Override
  public double getDriveDistanceMeters() {
    return m_driveEncoder.getPosition();
  }
}
