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
import edu.wpi.first.cameraserver.CameraServer;
import edu.wpi.first.cscore.UsbCamera;
import edu.wpi.first.hal.SimDouble;
import edu.wpi.first.hal.simulation.SimDeviceDataJNI;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.DifferentialDriveKinematics;
import edu.wpi.first.math.kinematics.DifferentialDriveOdometry;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
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
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.DriveConstants;
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

  DifferentialDrive m_drive;
  DifferentialDrivetrainSim m_driveSim;

  PowerDistribution pdp;

  PIDController turnPID;

  ShuffleboardTab driveTab;

  DifferentialDriveKinematics kinematics;
  DifferentialDriveOdometry odometry;

  SimpleMotorFeedforward feedforward;
  PIDController leftPID;
  PIDController rightPID;

  Field2d field;
  Pose2d pose;
  UsbCamera driverCam;

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

    leftEncoder.setPositionConversionFactor(
        (Units.inchesToMeters(Constants.DriveConstants.WHEEL_DIAMETER) * Math.PI)
            / Constants.DriveConstants.GEAR_RATIO);
    rightEncoder.setPositionConversionFactor(
        (Units.inchesToMeters(Constants.DriveConstants.WHEEL_DIAMETER) * Math.PI)
            / Constants.DriveConstants.GEAR_RATIO);

    leftMotors = new MotorControllerGroup(leftMaster, leftSlave1, leftSlave2);
    rightMotors = new MotorControllerGroup(rightMaster, rightSlave1, rightSlave2);

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
    turnPID.setTolerance(Constants.DriveConstants.QUICK_TURN_TOLERANCE);

    driveTab.add("Turn PID", turnPID).withWidget(BuiltInWidgets.kPIDController);

    // invertMotors(false);
    
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

    // Camera
    driverCam = CameraServer.startAutomaticCapture();
    driveTab.add("Driver Cam", driverCam).withWidget(BuiltInWidgets.kCameraStream);
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
   * @param left The speed of the left side of the robot.
   * @param right The speed of the right side of the robot.
   */
  public void tankDrive(double left, double right) {
    m_drive.tankDrive(left * DriveConstants.DRIVE_SPEED, right * DriveConstants.DRIVE_SPEED);
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

  // public double getDirection() {
  //   double leftV = leftEncoder.getVelocity();
  //   double rightV = -rightEncoder.getVelocity();

  //   SmartDashboard.putNumber("Left V", leftV);
  //   SmartDashboard.putNumber("Right V", rightV);

  //   double average = (leftV + rightV) / 2.0;
  //   SmartDashboard.putNumber("Average V", average);
  //   SmartDashboard.putBoolean("Direction", average > .1);

  //   return average;
  // }

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
            gyro.getRotation2d(), leftEncoder.getPosition(), rightEncoder.getPosition());

    field.setRobotPose(pose);

    SmartDashboard.putData("Power Distribution", pdp);
    SmartDashboard.putNumber("Current Angle", getAngleBetween(getHeading(), 0));
  }

  @Override
  public void simulationPeriodic() {
    m_driveSim.setInputs(
        leftMotors.get() * RobotController.getInputVoltage(),
        rightMotors.get() * RobotController.getInputVoltage());
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

  public void invertMotors(boolean inverted) {
    leftMaster.setInverted(inverted);
    rightMaster.setInverted(inverted);
    leftSlave1.setInverted(inverted);
    leftSlave2.setInverted(inverted);
    rightSlave1.setInverted(inverted);
    rightSlave2.setInverted(inverted);
  }
}
