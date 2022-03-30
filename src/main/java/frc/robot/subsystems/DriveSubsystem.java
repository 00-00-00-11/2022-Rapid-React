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
import edu.wpi.first.math.trajectory.Trajectory;
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
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.Constants;
import frc.robot.Robot;
import frc.robot.RobotContainer;
import frc.robot.Constants.DriveConstants;
import frc.robot.commands.IntakeAndIndex;
import frc.robot.utility.RamseteUtility;
import frc.robot.utility.TrajectoryUtility;
import frc.robot.utility.SparkMaxUtility;

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
  RelativeEncoder leftEncoder2;
  RelativeEncoder rightEncoder2;
  RelativeEncoder leftEncoder3;
  RelativeEncoder rightEncoder3;

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

  SendableChooser<Integer> m_chooser;
  int simInvert;

  public DriveSubsystem() {
    gyro = new AHRS(SPI.Port.kMXP);

    rightMaster = SparkMaxUtility.constructSparkMax(Constants.RobotMap.RIGHT_MASTER_CAN, true);
    rightSlave1 = SparkMaxUtility.constructSparkMax(Constants.RobotMap.RIGHT_SLAVE_CAN1, true);
    rightSlave2 = SparkMaxUtility.constructSparkMax(Constants.RobotMap.RIGHT_SLAVE_CAN2, true);

    leftMaster = SparkMaxUtility.constructSparkMax(Constants.RobotMap.LEFT_MASTER_CAN, true);
    leftSlave1 = SparkMaxUtility.constructSparkMax(Constants.RobotMap.LEFT_SLAVE_CAN1, true);
    leftSlave2 = SparkMaxUtility.constructSparkMax(Constants.RobotMap.LEFT_SLAVE_CAN2, true);

    leftEncoder = leftMaster.getEncoder();
    rightEncoder = rightMaster.getEncoder();
    leftEncoder2 = leftSlave1.getEncoder();
    rightEncoder2 = rightSlave1.getEncoder();
    leftEncoder3 = leftSlave2.getEncoder();
    rightEncoder3 = rightSlave2.getEncoder();

    leftEncoder.setPosition(0d);
    rightEncoder.setPosition(0d);
    leftEncoder2.setPosition(0d);
    rightEncoder2.setPosition(0d);
    leftEncoder3.setPosition(0d);
    rightEncoder3.setPosition(0d);



    leftEncoder.setPositionConversionFactor(
        (Units.inchesToMeters(Constants.DriveConstants.WHEEL_DIAMETER) * Math.PI)
            / Constants.DriveConstants.GEAR_RATIO);
    rightEncoder.setPositionConversionFactor(
        (Units.inchesToMeters(Constants.DriveConstants.WHEEL_DIAMETER) * Math.PI)
            / Constants.DriveConstants.GEAR_RATIO);
    leftEncoder2.setPositionConversionFactor(
        (Units.inchesToMeters(Constants.DriveConstants.WHEEL_DIAMETER) * Math.PI)
            / Constants.DriveConstants.GEAR_RATIO);
    rightEncoder2.setPositionConversionFactor(
        (Units.inchesToMeters(Constants.DriveConstants.WHEEL_DIAMETER) * Math.PI)
            / Constants.DriveConstants.GEAR_RATIO);
    leftEncoder3.setPositionConversionFactor(
        (Units.inchesToMeters(Constants.DriveConstants.WHEEL_DIAMETER) * Math.PI)
            / Constants.DriveConstants.GEAR_RATIO);
    rightEncoder3.setPositionConversionFactor(
        (Units.inchesToMeters(Constants.DriveConstants.WHEEL_DIAMETER) * Math.PI)
            / Constants.DriveConstants.GEAR_RATIO);


    leftMotors = new MotorControllerGroup(leftMaster, leftSlave1, leftSlave2);
    rightMotors = new MotorControllerGroup(rightMaster, rightSlave1, rightSlave2);

    m_drive = new DifferentialDrive(leftMotors, rightMotors);

    field = new Field2d();

    driveTab = Shuffleboard.getTab("Drive");   
    driveTab.add("Gyro", gyro).withWidget(BuiltInWidgets.kGyro);
    driveTab.add("Field View", field).withWidget("Field");

    turnPID = new PIDController(
        Constants.DriveConstants.turnKP,
        Constants.DriveConstants.turnKI,
        Constants.DriveConstants.turnKD);

    turnPID.setSetpoint(0);
    turnPID.setTolerance(Constants.DriveConstants.QUICK_TURN_TOLERANCE);

    ultrasonicDist = driveTab.add("Distance to target", 500).getEntry();

    driveTab.add("Turn PID", turnPID).withWidget(BuiltInWidgets.kPIDController);
    leftMotors.setInverted(false);
    rightMotors.setInverted(true);
    setBrake(true);

    m_driveSim =
        new DifferentialDrivetrainSim(
            DCMotor.getNEO(3),
            Constants.DriveConstants.GEAR_RATIO,
            Constants.DriveConstants.jKg_METERS_SQUARED,
            DriveConstants.ROBOT_MASS,
            Units.inchesToMeters(DriveConstants.WHEEL_DIAMETER / 2),
            DriveConstants.TRACK_WIDTH,
            null);

    kinematics = new DifferentialDriveKinematics(Constants.DriveConstants.TRACK_WIDTH);
    odometry = new DifferentialDriveOdometry(Rotation2d.fromDegrees(getHeading()));

    feedforward = new SimpleMotorFeedforward(
        Constants.DriveConstants.ksVolts,
        Constants.DriveConstants.kvVoltSecondsPerMeter,
        Constants.DriveConstants.kaVoltSecondsSquaredPerMeter);
    leftPID = new PIDController(
        Constants.DriveConstants.kP, Constants.DriveConstants.kI, Constants.DriveConstants.kD);
    rightPID = new PIDController(
        Constants.DriveConstants.kP, Constants.DriveConstants.kI, Constants.DriveConstants.kD);

    m_chooser = new SendableChooser<>();
    m_chooser.addOption("4 Ball Routine", 4);
    m_chooser.addOption("3 Ball Routine", 3);
    m_chooser.addOption("2 Ball Routine", 2);
    m_chooser.setDefaultOption("Exit Tarmac", 0);
    SmartDashboard.putData("Auto Routine Chooser", m_chooser);

    simInvert = Robot.isReal() ? -1 : 1;

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
   * Controls each side of the robot individually.
   *
   * @param left  The speed of the left side of the robot.
   * @param right The speed of the right side of the robot.
   *//////////////
  public void tankDrive(double left, double right) {
    m_drive.tankDrive(left * DriveConstants.DRIVE_SPEED, right * DriveConstants.DRIVE_SPEED);
  }

  public void tankDriveAuto(double left, double right) {
    m_drive.tankDrive(left, right);
  }

  /**
   * Controls the robot with curveDrive.
   *
   * @param xSpeed   The robot's speed along the X axis [-1.0..1.0]. Forward is
   *                 positive.
   * @param rotation The robot's rotation rate around the Z axis [-1.0..1.0].
   *                 Clockwise is positive.
   * @param turn     Whether or not to turn in place.
   */
  public void curveDrive(double xSpeed, double rotation, boolean turn) {
    m_drive.curvatureDrive(
        xSpeed * DriveConstants.DRIVE_SPEED, rotation * DriveConstants.TURN_SPEED, turn);
  }

  public void arcadeDrive(double xSpeed, double rotation, boolean stabilize) {
    m_drive.arcadeDrive(
        xSpeed * DriveConstants.DRIVE_SPEED, rotation * DriveConstants.TURN_SPEED, stabilize);
  }

  /**
   * Controls the left and right sides of the drive directly with voltages.
   *
   * @param leftVolts  the commanded left output
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

  // public double getDirection() {
  // double leftV = leftEncoder.getVelocity();
  // double rightV = -rightEncoder.getVelocity();

  // SmartDashboard.putNumber("Left V", leftV);
  // SmartDashboard.putNumber("Right V", rightV);

  // double average = (leftV + rightV) / 2.0;
  // SmartDashboard.putNumber("Average V", average);
  // SmartDashboard.putBoolean("Direction", average > .1);

  // return average;
  // }

  public double getAngleBetween(double current, double target) {
    double degrees = target - current;
    degrees %= 360;
    if (degrees > 180)
      degrees -= 360;
    if (degrees < -180)
      degrees += 360;
    return degrees;
  }

  public PIDController getTurnPID() {
    return turnPID;
  }

  @Override
  public void periodic() {

    pose = odometry.update(
        gyro.getRotation2d(), leftEncoder.getPosition(), rightEncoder.getPosition() * simInvert);

    field.setRobotPose(pose);
    
    SmartDashboard.putNumber("Current Angle", getAngleBetween(getHeading(), 0));
    SmartDashboard.putNumber("Motor 4: ", leftEncoder.getVelocity());
    SmartDashboard.putNumber("Motor 1: ", rightEncoder.getVelocity());
    SmartDashboard.putNumber("Motor 5: ", leftEncoder2.getVelocity());
    SmartDashboard.putNumber("Motor 6: ", leftEncoder3.getVelocity());
    SmartDashboard.putNumber("Motor 2: ", rightEncoder2.getVelocity());
    SmartDashboard.putNumber("Motor 3: ", rightEncoder3.getVelocity());

  }

  @Override
  public void simulationPeriodic() {
    m_driveSim.setInputs(
        // Left and right CAN IDs are flipped on the robot so right and left sides are
        // flipped for
        // simulation
        rightMotors.get() * RobotController.getInputVoltage(),
        leftMotors.get() * RobotController.getInputVoltage());
    m_driveSim.update(0.02);

    leftEncoder.setPosition(m_driveSim.getLeftPositionMeters());
    rightEncoder.setPosition(m_driveSim.getRightPositionMeters());

    int leftHandle = SimDeviceDataJNI.getSimDeviceHandle("SPARK MAX [" + leftMaster.getDeviceId() + "]");
    int rightHandle = SimDeviceDataJNI.getSimDeviceHandle("SPARK MAX [" + rightMaster.getDeviceId() + "]");
    SimDouble leftVelocity = new SimDouble(SimDeviceDataJNI.getSimValueHandle(leftHandle, "Velocity"));
    SimDouble rightVelocity = new SimDouble(SimDeviceDataJNI.getSimValueHandle(rightHandle, "Velocity"));
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
  }

  public double getEncoderPosition() {
    return leftEncoder.getPosition();
  }

  public Pose2d getPose() {
    return pose;
  }

  public DifferentialDriveWheelSpeeds getSpeeds() {
    return new DifferentialDriveWheelSpeeds(
        leftMaster.getEncoder().getVelocity() / Constants.DriveConstants.GEAR_RATIO * 2 * Math.PI
            * Units.inchesToMeters(3.0) / 60,
        rightMaster.getEncoder().getVelocity() / Constants.DriveConstants.GEAR_RATIO * 2 * Math.PI
            * Units.inchesToMeters(3.0) / 60);
  }

  public Integer getSelectedFromChooser() {
    return m_chooser.getSelected();
  }

  public SequentialCommandGroup FourBallAuto(DriveSubsystem drive) {
    Trajectory traj1 = TrajectoryUtility.createNewTrajectoryFromJSON("4Ball-1");
    Trajectory traj2 = TrajectoryUtility.createNewTrajectoryFromJSON("4Ball-2");
    Trajectory traj3 = TrajectoryUtility.createNewTrajectoryFromJSON("4Ball-3");
    Trajectory traj4 = TrajectoryUtility.createNewTrajectoryFromJSON("4Ball-4");
    resetOdometry(traj1.getInitialPose());

    long startTime = System.currentTimeMillis();

    return new SequentialCommandGroup(

        new InstantCommand(
            () -> {
              RobotContainer.m_intakeSubsystem.forwardIntake();
              SmartDashboard.putString("AUTO STATUS", "EXTENDED INTAKE");
            }),

        new ParallelRaceGroup(
            RamseteUtility.createRamseteCommand(traj1, RobotContainer.m_driveSubsystem, false)
                .andThen(() -> RobotContainer.m_driveSubsystem.setOutput(0, 0)),
            new IntakeAndIndex()),

        new InstantCommand(
            () -> {
              RobotContainer.m_intakeSubsystem.reverseIntake();
              SmartDashboard.putString("AUTO STATUS", "RETRACTED INTAKE");
            }),

        RamseteUtility.createRamseteCommand(traj2, RobotContainer.m_driveSubsystem, false)
            .andThen(() -> RobotContainer.m_driveSubsystem.setOutput(0, 0)),

        new InstantCommand(
            () -> {
              // RobotContainer.m_shooter_subsystem.shootCIM(Constants.ShooterConstants.kCIMSpeed);
              SmartDashboard.putString("AUTO STATUS", "SHOOTING");
            }),

        new WaitCommand(2),

        new InstantCommand(
            () -> {
              // RobotContainer.m_shooter_subsystem.shootCIM(0);
              SmartDashboard.putString("AUTO STATUS", "STOPPED SHOOTING");
            }),

        new InstantCommand(
            () -> {
              RobotContainer.m_intakeSubsystem.forwardIntake();
              SmartDashboard.putString("AUTO STATUS", "EXTENDED INTAKE");
            }),

        RamseteUtility.createRamseteCommand(traj3, RobotContainer.m_driveSubsystem, false)
            .andThen(() -> RobotContainer.m_driveSubsystem.setOutput(0, 0)),

        new InstantCommand(
            () -> {
              RobotContainer.m_intakeSubsystem.reverseIntake();
              SmartDashboard.putString("AUTO STATUS", "RETRACTED INTAKE");
            }),

        RamseteUtility.createRamseteCommand(traj4, RobotContainer.m_driveSubsystem, false)
            .andThen(() -> RobotContainer.m_driveSubsystem.setOutput(0, 0)),

        new InstantCommand(
            () -> {
              // RobotContainer.m_shooter_subsystem.shootCIM(Constants.ShooterConstants.kCIMSpeed);
              SmartDashboard.putString("AUTO STATUS", "SHOOTING");
            }),

        new WaitCommand(2),

        new InstantCommand(
            () -> {
              // RobotContainer.m_shooter_subsystem.shootCIM(0);
              SmartDashboard.putString("AUTO STATUS", "STOPPED SHOOTING");
            }),

        new InstantCommand(
            () -> {
              long elapsedTime = System.currentTimeMillis() - startTime;
              SmartDashboard.putNumber("AUTO TIME", elapsedTime / 1000);
            })

    );
  }

  public SequentialCommandGroup ThreeBallAuto(DriveSubsystem drive) {
    Trajectory traj1 = TrajectoryUtility.createNewTrajectoryFromJSON("3Ball-1");
    Trajectory traj2 = TrajectoryUtility.createNewTrajectoryFromJSON("3Ball-2");
    Trajectory traj3 = TrajectoryUtility.createNewTrajectoryFromJSON("3Ball-3");
    Trajectory traj4 = TrajectoryUtility.createNewTrajectoryFromJSON("3Ball-4");
    resetOdometry(traj1.getInitialPose());

    return new SequentialCommandGroup(

        new InstantCommand(
            () -> {
              RobotContainer.m_intakeSubsystem.forwardIntake();
              SmartDashboard.putString("AUTO STATUS", "EXTENDED INTAKE");
            }),

        new ParallelRaceGroup(
            RamseteUtility.createRamseteCommand(traj1, RobotContainer.m_driveSubsystem, false)
                .andThen(() -> RobotContainer.m_driveSubsystem.setOutput(0, 0)),
            new IntakeAndIndex()),

        new InstantCommand(
            () -> {
              RobotContainer.m_intakeSubsystem.reverseIntake();
              SmartDashboard.putString("AUTO STATUS", "RETRACTED INTAKE");
            }),

        new InstantCommand(
            () -> {
              RobotContainer.m_intakeSubsystem.forwardIntake();
              SmartDashboard.putString("AUTO STATUS", "EXTENDED INTAKE");
            }),

        new ParallelRaceGroup(
            RamseteUtility.createRamseteCommand(traj2, RobotContainer.m_driveSubsystem, false)
                .andThen(() -> RobotContainer.m_driveSubsystem.setOutput(0, 0)),
            new IntakeAndIndex()),

        new InstantCommand(
            () -> {
              RobotContainer.m_intakeSubsystem.reverseIntake();
              SmartDashboard.putString("AUTO STATUS", "RETRACTED INTAKE");
            }),

        RamseteUtility.createRamseteCommand(traj3, RobotContainer.m_driveSubsystem, false)
            .andThen(() -> RobotContainer.m_driveSubsystem.setOutput(0, 0)),

        new InstantCommand(
            () -> {
              // RobotContainer.m_shooter_subsystem.shootCIM(Constants.ShooterConstants.kCIMSpeed);
              SmartDashboard.putString("AUTO STATUS", "SHOOTING");
            }),

        new WaitCommand(2),

        new InstantCommand(
            () -> {
              // RobotContainer.m_shooter_subsystem.shootCIM(0);
              SmartDashboard.putString("AUTO STATUS", "STOPPED SHOOTING");
            }),

        new InstantCommand(
            () -> {
              RobotContainer.m_intakeSubsystem.forwardIntake();
              SmartDashboard.putString("AUTO STATUS", "EXTENDED INTAKE");
            }),

        new ParallelRaceGroup(
            RamseteUtility.createRamseteCommand(traj4, RobotContainer.m_driveSubsystem, false)
                .andThen(() -> RobotContainer.m_driveSubsystem.setOutput(0, 0))),
        new IntakeAndIndex());

  }

  public SequentialCommandGroup TwoBallAuto(DriveSubsystem drive) {
    Trajectory traj1 = TrajectoryUtility.createNewTrajectoryFromJSON("2Ball-1");
    Trajectory traj2 = TrajectoryUtility.createNewTrajectoryFromJSON("2Ball-2");
    Trajectory traj3 = TrajectoryUtility.createNewTrajectoryFromJSON("2Ball-3");
    Trajectory traj4 = TrajectoryUtility.createNewTrajectoryFromJSON("2Ball-4");
    resetOdometry(traj1.getInitialPose());

    return new SequentialCommandGroup(
        RamseteUtility.createRamseteCommand(traj1, RobotContainer.m_driveSubsystem, false)
            .andThen(() -> RobotContainer.m_driveSubsystem.setOutput(0, 0)).withTimeout(15),
        RamseteUtility.createRamseteCommand(traj2, RobotContainer.m_driveSubsystem, false)
            .andThen(() -> RobotContainer.m_driveSubsystem.setOutput(0, 0)).withTimeout(15),
        RamseteUtility.createRamseteCommand(traj3, RobotContainer.m_driveSubsystem, false)
            .andThen(() -> RobotContainer.m_driveSubsystem.setOutput(0, 0)).withTimeout(15),
        RamseteUtility.createRamseteCommand(traj4, RobotContainer.m_driveSubsystem, false)
            .andThen(() -> RobotContainer.m_driveSubsystem.setOutput(0, 0)).withTimeout(15));
  }

}
