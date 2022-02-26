/***
 *
 *                                                                                                                                      bbbbbbbb
 *    DDDDDDDDDDDDD                             iiii                                                   SSSSSSSSSSSSSSS                  b::::::b                                                                           tttt
 *    D::::::::::::DDD                         i::::i                                                SS:::::::::::::::S                 b::::::b                                                                        ttt:::t
 *    D:::::::::::::::DD                        iiii                                                S:::::SSSSSS::::::S                 b::::::b                                                                        t:::::t
 *    DDD:::::DDDDD:::::D                                                                           S:::::S     SSSSSSS                  b:::::b                                                                        t:::::t
 *      D:::::D    D:::::D rrrrr   rrrrrrrrr  iiiiiiivvvvvvv           vvvvvvv eeeeeeeeeeee         S:::::S            uuuuuu    uuuuuu  b:::::bbbbbbbbb        ssssssssssyyyyyyy           yyyyyyy  ssssssssss   ttttttt:::::ttttttt        eeeeeeeeeeee       mmmmmmm    mmmmmmm
 *      D:::::D     D:::::Dr::::rrr:::::::::r i:::::i v:::::v         v:::::vee::::::::::::ee       S:::::S            u::::u    u::::u  b::::::::::::::bb    ss::::::::::sy:::::y         y:::::y ss::::::::::s  t:::::::::::::::::t      ee::::::::::::ee   mm:::::::m  m:::::::mm
 *      D:::::D     D:::::Dr:::::::::::::::::r i::::i  v:::::v       v:::::ve::::::eeeee:::::ee      S::::SSSS         u::::u    u::::u  b::::::::::::::::b ss:::::::::::::sy:::::y       y:::::yss:::::::::::::s t:::::::::::::::::t     e::::::eeeee:::::eem::::::::::mm::::::::::m
 *      D:::::D     D:::::Drr::::::rrrrr::::::ri::::i   v:::::v     v:::::ve::::::e     e:::::e       SS::::::SSSSS    u::::u    u::::u  b:::::bbbbb:::::::bs::::::ssss:::::sy:::::y     y:::::y s::::::ssss:::::stttttt:::::::tttttt    e::::::e     e:::::em::::::::::::::::::::::m
 *      D:::::D     D:::::D r:::::r     r:::::ri::::i    v:::::v   v:::::v e:::::::eeeee::::::e         SSS::::::::SS  u::::u    u::::u  b:::::b    b::::::b s:::::s  ssssss  y:::::y   y:::::y   s:::::s  ssssss       t:::::t          e:::::::eeeee::::::em:::::mmm::::::mmm:::::m
 *      D:::::D     D:::::D r:::::r     rrrrrrri::::i     v:::::v v:::::v  e:::::::::::::::::e             SSSSSS::::S u::::u    u::::u  b:::::b     b:::::b   s::::::s        y:::::y y:::::y      s::::::s            t:::::t          e:::::::::::::::::e m::::m   m::::m   m::::m
 *      D:::::D     D:::::D r:::::r            i::::i      v:::::v:::::v   e::::::eeeeeeeeeee                   S:::::Su::::u    u::::u  b:::::b     b:::::b      s::::::s      y:::::y:::::y          s::::::s         t:::::t          e::::::eeeeeeeeeee  m::::m   m::::m   m::::m
 *      D:::::D    D:::::D  r:::::r            i::::i       v:::::::::v    e:::::::e                            S:::::Su:::::uuuu:::::u  b:::::b     b:::::bssssss   s:::::s     y:::::::::y     ssssss   s:::::s       t:::::t    tttttte:::::::e           m::::m   m::::m   m::::m
 *    DDD:::::DDDDD:::::D   r:::::r           i::::::i       v:::::::v     e::::::::e               SSSSSSS     S:::::Su:::::::::::::::uub:::::bbbbbb::::::bs:::::ssss::::::s     y:::::::y      s:::::ssss::::::s      t::::::tttt:::::te::::::::e          m::::m   m::::m   m::::m
 *    D:::::::::::::::DD    r:::::r           i::::::i        v:::::v       e::::::::eeeeeeee       S::::::SSSSSS:::::S u:::::::::::::::ub::::::::::::::::b s::::::::::::::s       y:::::y       s::::::::::::::s       tt::::::::::::::t e::::::::eeeeeeee  m::::m   m::::m   m::::m
 *    D::::::::::::DDD      r:::::r           i::::::i         v:::v         ee:::::::::::::e       S:::::::::::::::SS   uu::::::::uu:::ub:::::::::::::::b   s:::::::::::ss       y:::::y         s:::::::::::ss          tt:::::::::::tt  ee:::::::::::::e  m::::m   m::::m   m::::m
 *    DDDDDDDDDDDDD         rrrrrrr           iiiiiiii          vvv            eeeeeeeeeeeeee        SSSSSSSSSSSSSSS       uuuuuuuu  uuuubbbbbbbbbbbbbbbb     sssssssssss        y:::::y           sssssssssss              ttttttttttt      eeeeeeeeeeeeee  mmmmmm   mmmmmm   mmmmmm
 *                                                                                                                                                                              y:::::y
 *                                                                                                                                                                             y:::::y
 *                                                                                                                                                                            y:::::y
 *                                                                                                                                                                           y:::::y
 *                                                                                                                                                                          yyyyyyy
 */

package frc.robot.subsystems;

import com.kauailabs.navx.frc.AHRS;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.IdleMode;
import com.revrobotics.CANSparkMaxLowLevel;
import com.revrobotics.RelativeEncoder;
import edu.wpi.first.hal.SimDouble;
import edu.wpi.first.hal.simulation.SimDeviceDataJNI;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.kinematics.DifferentialDriveWheelSpeeds;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.AnalogInput;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.SPI;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.MotorControllerGroup;
import edu.wpi.first.wpilibj.shuffleboard.BuiltInWidgets;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj.simulation.DifferentialDrivetrainSim;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.RobotContainer;
import frc.robot.Constants.DriveConstants;
import frc.robot.commands.ExitTarmac;
import frc.robot.utility.RamseteUtility;
import frc.robot.utility.TrajectoryUtility;

public class DriveSubsystem extends SubsystemBase {

  AHRS gyro;

  CANSparkMax leftMaster;
  CANSparkMax leftSlave1;
  CANSparkMax leftSlave2;

  CANSparkMax rightMaster;
  CANSparkMax rightSlave1;
  CANSparkMax rightSlave2;

  MotorControllerGroup leftMotors;
  MotorControllerGroup rightMotors;

  RelativeEncoder leftEncoder;
  RelativeEncoder rightEncoder;

  DifferentialDrive m_drive;
  DifferentialDrivetrainSim m_driveSim;

  PowerDistribution pdp;

  PIDController turnPID;

  ShuffleboardTab driveTab;

  DifferentialDriveKinematics kinematics;
  DifferentialDriveOdometry odometry;

  AnalogInput ultrasonic;
  double raw_value;
  double voltage_scale_factor;
  double currentDistanceCentimeters;
  NetworkTableEntry ultrasonicDist;

  SimpleMotorFeedforward feedforward;
  PIDController leftPID;
  PIDController rightPID;

  public Field2d field;
  Pose2d pose;

  SendableChooser<Command> m_chooser;

  // Camera

  public DriveSubsystem() {
    gyro = new AHRS(SPI.Port.kMXP);

    rightMaster = contructSpeedController(Constants.DriveConstants.RIGHT_MASTER_CAN, true);
    rightSlave1 = contructSpeedController(Constants.DriveConstants.RIGHT_SLAVE_CAN1, true);
    rightSlave2 = contructSpeedController(Constants.DriveConstants.RIGHT_SLAVE_CAN2, true);

    leftMaster = contructSpeedController(Constants.DriveConstants.LEFT_MASTER_CAN, true);
    leftSlave1 = contructSpeedController(Constants.DriveConstants.LEFT_SLAVE_CAN1, true);
    leftSlave2 = contructSpeedController(Constants.DriveConstants.LEFT_SLAVE_CAN2, true);

    leftEncoder = leftMaster.getEncoder();
    rightEncoder = rightMaster.getEncoder();

    leftEncoder.setPosition(0d);
    rightEncoder.setPosition(0d);

    leftEncoder.setPositionConversionFactor(
        (Units.inchesToMeters(Constants.DriveConstants.WHEEL_DIAMETER) * Math.PI)
            / Constants.DriveConstants.GEAR_RATIO);
    rightEncoder.setPositionConversionFactor(
        (Units.inchesToMeters(Constants.DriveConstants.WHEEL_DIAMETER) * Math.PI)
            / Constants.DriveConstants.GEAR_RATIO);

    leftMotors = new MotorControllerGroup(leftMaster, leftSlave1, leftSlave2);
    rightMotors = new MotorControllerGroup(rightMaster, rightSlave1, rightSlave2);

    ultrasonic = new AnalogInput(1);

    m_drive = new DifferentialDrive(leftMotors, rightMotors);

    pdp = new PowerDistribution();

    field = new Field2d();

    driveTab = Shuffleboard.getTab("Drive");
    driveTab.add("Differential Drive", m_drive).withWidget(BuiltInWidgets.kDifferentialDrive);
    driveTab.add("Gyro", gyro).withWidget(BuiltInWidgets.kGyro);
    driveTab.add("Field View", field).withWidget("Field");
    driveTab.add("Power Distribution Panel", pdp).withWidget(BuiltInWidgets.kPowerDistribution);

    turnPID =
        new PIDController(
            Constants.DriveConstants.turnKP,
            Constants.DriveConstants.turnKI,
            Constants.DriveConstants.turnKD);

    turnPID.setSetpoint(0);
    turnPID.setTolerance(5);

    ultrasonicDist = driveTab.add("Distance to target", 500).getEntry();

    driveTab.add("Turn PID", turnPID).withWidget(BuiltInWidgets.kPIDController);

    rightMotors.setInverted(true);
    setBrake(true);

    SmartDashboard.putBoolean("Valet Mode", false);

    m_driveSim =
        new DifferentialDrivetrainSim(
            DCMotor.getNEO(3),
            Constants.DriveConstants.GEAR_RATIO,
            Constants.DriveConstants.jKg_METERS_SQUARED,
            DriveConstants.ROBOT_MASS,
            Units.inchesToMeters(DriveConstants.WHEEL_DIAMETER / 2),
            DriveConstants.TRACK_WIDTH,
            null);
    // VecBuilder.fill(0.001, 0.001, 0.001, 0.1, 0.1, 0.005, 0.005));

    kinematics = new DifferentialDriveKinematics(Constants.DriveConstants.TRACK_WIDTH);
    odometry = new DifferentialDriveOdometry(Rotation2d.fromDegrees(getHeading()));

    feedforward =
        new SimpleMotorFeedforward(
            Constants.DriveConstants.ksVolts,
            Constants.DriveConstants.kvVoltSecondsPerMeter,
            Constants.DriveConstants.kaVoltSecondsSquaredPerMeter);
    leftPID =
        new PIDController(
            Constants.DriveConstants.kP, Constants.DriveConstants.kI, Constants.DriveConstants.kD);
    rightPID =
        new PIDController(
            Constants.DriveConstants.kP, Constants.DriveConstants.kI, Constants.DriveConstants.kD);

    m_chooser = new SendableChooser<>();
    m_chooser.addOption("4 Ball Routine", FourBallAuto(RobotContainer.m_driveSubsystem));
    m_chooser.addOption("3 Ball Routine", ThreeBallAuto(RobotContainer.m_driveSubsystem));
    m_chooser.addOption("2 Ball Routine", TwoBallAuto(RobotContainer.m_driveSubsystem));
    m_chooser.setDefaultOption("Exit Tarmac", new ExitTarmac());
    SmartDashboard.putData("Auto Routine Chooser", m_chooser);
  }

  /**
   * Gets the motors in the subsystem
   *
   * @return the motors
   */
  public CANSparkMax[] getMotors() {
    return new CANSparkMax[] {
      leftMaster, leftSlave1, leftSlave2, rightMaster, rightSlave1, rightSlave2
    };
  }

  /**
   * Sets the default brake mode for the drivetrain.
   *
   * @param brake Whether or not to use brake mode.
   */
  public void setBrake(boolean on) {
    IdleMode mode = on ? IdleMode.kBrake : IdleMode.kCoast;
    CANSparkMax[] motors = getMotors();
    for (CANSparkMax motor : motors) {
      motor.setIdleMode(mode);
    }
  }

  /**
   * Constructs a CANSparkMax with the given CAN ID and type.
   *
   * @param port CAN ID of the motor controller.
   * @param brushless Whether the motor is brushless.
   * @return CANSparkMax object.
   */
  public CANSparkMax contructSpeedController(int port, boolean brushless) {
    CANSparkMaxLowLevel.MotorType type =
        brushless
            ? CANSparkMaxLowLevel.MotorType.kBrushless
            : CANSparkMaxLowLevel.MotorType.kBrushed;
    return new CANSparkMax(port, type);
  }

  /**
   * Controls each side of the robot individually.
   *
   * @param left The speed of the left side of the robot.
   * @param right The speed of the right side of the robot.
   */
  public void tankDrive(double left, double right) {
    m_drive.tankDrive(left * DriveConstants.DRIVE_SPEED, right * DriveConstants.DRIVE_SPEED);
  }

  public void tankDriveAuto(double left, double right) {
    m_drive.tankDrive(left, right);
  }

  /**
   * Controls the robot with curveDrive.
   *
   * @param xSpeed The robot's speed along the X axis [-1.0..1.0]. Forward is positive.
   * @param rotation The robot's rotation rate around the Z axis [-1.0..1.0]. Clockwise is positive.
   * @param turn Whether or not to turn in place.
   */
  public void curveDrive(double xSpeed, double rotation, boolean turn) {
    m_drive.curvatureDrive(
        xSpeed * DriveConstants.DRIVE_SPEED, rotation * DriveConstants.TURN_SPEED, turn);
  }

  /**
   * Controls the left and right sides of the drive directly with voltages.
   *
   * @param leftVolts the commanded left output
   * @param rightVolts the commanded right output
   */
  public void tankDriveVolts(double leftVolts, double rightVolts) {
    leftMotors.setVoltage(leftVolts);
    rightMotors.setVoltage(rightVolts);
    m_drive.feed();
  }

  /** Resets the NavX to 0 degrees. */
  public void resetGyro() {
    gyro.zeroYaw();
  }

  /**
   * Returns the current heading of the robot in degrees.
   *
   * @return The current heading of the robot in degrees.
   */
  public double getHeading() {
    return gyro.getAngle();
  }

  public double getDirection() {
    double leftV = leftEncoder.getVelocity();
    double rightV = rightEncoder.getVelocity();

    SmartDashboard.putNumber("Left V", leftV);
    SmartDashboard.putNumber("Right V", rightV);

    double average = (leftV + rightV) / 2.0;
    SmartDashboard.putNumber("Average V", average);
    SmartDashboard.putBoolean("Direction", average > .1);

    return average;
  }

  public double getAngleBetween(double current, double target) {
    double degrees = target - current;
    degrees %= 360;
    if (degrees > 180) degrees -= 360;
    if (degrees < -180) degrees += 360;
    return degrees;
  }

  public PIDController getTurnPID() {
    return turnPID;
  }

  @Override
  public void periodic() {
    pose =
        odometry.update(
            gyro.getRotation2d(), leftEncoder.getPosition(), -rightEncoder.getPosition());

    field.setRobotPose(pose);
    SmartDashboard.putNumber("left encoder", leftEncoder.getPosition());
    SmartDashboard.putNumber("right encoder", rightEncoder.getPosition());

    voltage_scale_factor = 5 / RobotController.getVoltage5V();
    currentDistanceCentimeters = ultrasonic.getValue() * voltage_scale_factor * .125;
  }

  @Override
  public void simulationPeriodic() {
    m_driveSim.setInputs(
        // Left and right CAN IDs are flipped on the robot so right and left sides are flipped for
        // simulation
        rightMotors.get() * RobotController.getInputVoltage(),
        leftMotors.get() * RobotController.getInputVoltage());
    m_driveSim.update(0.02);

    leftEncoder.setPosition(m_driveSim.getLeftPositionMeters());
    rightEncoder.setPosition(m_driveSim.getRightPositionMeters());

    int leftHandle =
        SimDeviceDataJNI.getSimDeviceHandle("SPARK MAX [" + leftMaster.getDeviceId() + "]");
    int rightHandle =
        SimDeviceDataJNI.getSimDeviceHandle("SPARK MAX [" + rightMaster.getDeviceId() + "]");
    SimDouble leftVelocity =
        new SimDouble(SimDeviceDataJNI.getSimValueHandle(leftHandle, "Velocity"));
    SimDouble rightVelocity =
        new SimDouble(SimDeviceDataJNI.getSimValueHandle(rightHandle, "Velocity"));
    leftVelocity.set(m_driveSim.getLeftVelocityMetersPerSecond());
    rightVelocity.set(m_driveSim.getRightVelocityMetersPerSecond());

    int gyroHandle = SimDeviceDataJNI.getSimDeviceHandle("navX-Sensor[0]");
    SimDouble angle = new SimDouble(SimDeviceDataJNI.getSimValueHandle(gyroHandle, "Yaw"));
    angle.set(m_driveSim.getHeading().getDegrees());
  }

  public SimpleMotorFeedforward getFeedforward() {
    return feedforward;
  }

  public PIDController getLeftPIDController() {
    return leftPID;
  }

  public PIDController getRightPIDController() {
    return rightPID;
  }

  public DifferentialDriveKinematics getKinematics() {
    return kinematics;
  }

  public void resetEncoders() {
    leftMaster.getEncoder().setPosition(0.0);
    rightMaster.getEncoder().setPosition(0.0);
  }

  public void resetOdometry(Pose2d pose) {
    resetEncoders();
    odometry.resetPosition(pose, gyro.getRotation2d());
  }

  public void setOutput(double leftVolts, double rightVolts) {
    leftMotors.set(leftVolts / 12);
    rightMotors.set(rightVolts / 12);

    SmartDashboard.putNumber("left Motor", leftVolts / 12);
    System.out.println("left Motor : " + leftVolts / 12);
    SmartDashboard.putNumber("rigt Motor", rightVolts / 12);
    System.out.println("right Motor: " + rightVolts / 12);
  }

  public double getEncoderPosition() {
    return leftEncoder.getPosition();
  }

  public Pose2d getPose() {
    return pose;
  }

  public DifferentialDriveWheelSpeeds getSpeeds() {
    return new DifferentialDriveWheelSpeeds(
      leftMaster.getEncoder().getVelocity() / Constants.DriveConstants.GEAR_RATIO * 2 * Math.PI * Units.inchesToMeters(3.0) / 60,
      rightMaster.getEncoder().getVelocity() / Constants.DriveConstants.GEAR_RATIO * 2 * Math.PI * Units.inchesToMeters(3.0) / 60
    );
  }

  public Command getSelectedFromChooser() {
    return m_chooser.getSelected();
  }

  public SequentialCommandGroup FourBallAuto(DriveSubsystem drive) {
    return new SequentialCommandGroup(
      RamseteUtility.createRamseteCommand(TrajectoryUtility.createNewTrajectoryFromJSON("4Ball-1"), drive, true),
      RamseteUtility.createRamseteCommand(TrajectoryUtility.createNewTrajectoryFromJSON("4Ball-2"), drive, false),
      RamseteUtility.createRamseteCommand(TrajectoryUtility.createNewTrajectoryFromJSON("4Ball-3"), drive, false),
      RamseteUtility.createRamseteCommand(TrajectoryUtility.createNewTrajectoryFromJSON("4Ball-4"), drive, false)      
    );
  }

  public SequentialCommandGroup ThreeBallAuto(DriveSubsystem drive) {
    return new SequentialCommandGroup(
      RamseteUtility.createRamseteCommand(TrajectoryUtility.createNewTrajectoryFromJSON("3Ball-1"), drive, true),
      RamseteUtility.createRamseteCommand(TrajectoryUtility.createNewTrajectoryFromJSON("3Ball-2"), drive, false),
      RamseteUtility.createRamseteCommand(TrajectoryUtility.createNewTrajectoryFromJSON("3Ball-3"), drive, false),
      RamseteUtility.createRamseteCommand(TrajectoryUtility.createNewTrajectoryFromJSON("3Ball-4"), drive, false)      
    );
  }

  public SequentialCommandGroup TwoBallAuto(DriveSubsystem drive) {
    return new SequentialCommandGroup(
      RamseteUtility.createRamseteCommand(TrajectoryUtility.createNewTrajectoryFromJSON("2Ball-1"), drive, true),
      RamseteUtility.createRamseteCommand(TrajectoryUtility.createNewTrajectoryFromJSON("2Ball-2"), drive, false),
      RamseteUtility.createRamseteCommand(TrajectoryUtility.createNewTrajectoryFromJSON("2Ball-3"), drive, false)
    );
  }
  
}