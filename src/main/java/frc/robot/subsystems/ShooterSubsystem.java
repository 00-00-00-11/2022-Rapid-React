/***
 *
 *                                                                                                                                                                                     bbbbbbbb
 *       SSSSSSSSSSSSSSS hhhhhhh                                                        tttt                                                          SSSSSSSSSSSSSSS                  b::::::b                                                                           tttt
 *     SS:::::::::::::::Sh:::::h                                                     ttt:::t                                                        SS:::::::::::::::S                 b::::::b                                                                        ttt:::t
 *    S:::::SSSSSS::::::Sh:::::h                                                     t:::::t                                                       S:::::SSSSSS::::::S                 b::::::b                                                                        t:::::t
 *    S:::::S     SSSSSSSh:::::h                                                     t:::::t                                                       S:::::S     SSSSSSS                  b:::::b                                                                        t:::::t
 *    S:::::S             h::::h hhhhh          ooooooooooo      ooooooooooo   ttttttt:::::ttttttt        eeeeeeeeeeee    rrrrr   rrrrrrrrr        S:::::S            uuuuuu    uuuuuu  b:::::bbbbbbbbb        ssssssssssyyyyyyy           yyyyyyy  ssssssssss   ttttttt:::::ttttttt        eeeeeeeeeeee       mmmmmmm    mmmmmmm
 *    S:::::S             h::::hh:::::hhh     oo:::::::::::oo  oo:::::::::::oo t:::::::::::::::::t      ee::::::::::::ee  r::::rrr:::::::::r       S:::::S            u::::u    u::::u  b::::::::::::::bb    ss::::::::::sy:::::y         y:::::y ss::::::::::s  t:::::::::::::::::t      ee::::::::::::ee   mm:::::::m  m:::::::mm
 *     S::::SSSS          h::::::::::::::hh  o:::::::::::::::oo:::::::::::::::ot:::::::::::::::::t     e::::::eeeee:::::eer:::::::::::::::::r       S::::SSSS         u::::u    u::::u  b::::::::::::::::b ss:::::::::::::sy:::::y       y:::::yss:::::::::::::s t:::::::::::::::::t     e::::::eeeee:::::eem::::::::::mm::::::::::m
 *      SS::::::SSSSS     h:::::::hhh::::::h o:::::ooooo:::::oo:::::ooooo:::::otttttt:::::::tttttt    e::::::e     e:::::err::::::rrrrr::::::r       SS::::::SSSSS    u::::u    u::::u  b:::::bbbbb:::::::bs::::::ssss:::::sy:::::y     y:::::y s::::::ssss:::::stttttt:::::::tttttt    e::::::e     e:::::em::::::::::::::::::::::m
 *        SSS::::::::SS   h::::::h   h::::::ho::::o     o::::oo::::o     o::::o      t:::::t          e:::::::eeeee::::::e r:::::r     r:::::r         SSS::::::::SS  u::::u    u::::u  b:::::b    b::::::b s:::::s  ssssss  y:::::y   y:::::y   s:::::s  ssssss       t:::::t          e:::::::eeeee::::::em:::::mmm::::::mmm:::::m
 *           SSSSSS::::S  h:::::h     h:::::ho::::o     o::::oo::::o     o::::o      t:::::t          e:::::::::::::::::e  r:::::r     rrrrrrr            SSSSSS::::S u::::u    u::::u  b:::::b     b:::::b   s::::::s        y:::::y y:::::y      s::::::s            t:::::t          e:::::::::::::::::e m::::m   m::::m   m::::m
 *                S:::::S h:::::h     h:::::ho::::o     o::::oo::::o     o::::o      t:::::t          e::::::eeeeeeeeeee   r:::::r                             S:::::Su::::u    u::::u  b:::::b     b:::::b      s::::::s      y:::::y:::::y          s::::::s         t:::::t          e::::::eeeeeeeeeee  m::::m   m::::m   m::::m
 *                S:::::S h:::::h     h:::::ho::::o     o::::oo::::o     o::::o      t:::::t    tttttte:::::::e            r:::::r                             S:::::Su:::::uuuu:::::u  b:::::b     b:::::bssssss   s:::::s     y:::::::::y     ssssss   s:::::s       t:::::t    tttttte:::::::e           m::::m   m::::m   m::::m
 *    SSSSSSS     S:::::S h:::::h     h:::::ho:::::ooooo:::::oo:::::ooooo:::::o      t::::::tttt:::::te::::::::e           r:::::r                 SSSSSSS     S:::::Su:::::::::::::::uub:::::bbbbbb::::::bs:::::ssss::::::s     y:::::::y      s:::::ssss::::::s      t::::::tttt:::::te::::::::e          m::::m   m::::m   m::::m
 *    S::::::SSSSSS:::::S h:::::h     h:::::ho:::::::::::::::oo:::::::::::::::o      tt::::::::::::::t e::::::::eeeeeeee   r:::::r                 S::::::SSSSSS:::::S u:::::::::::::::ub::::::::::::::::b s::::::::::::::s       y:::::y       s::::::::::::::s       tt::::::::::::::t e::::::::eeeeeeee  m::::m   m::::m   m::::m
 *    S:::::::::::::::SS  h:::::h     h:::::h oo:::::::::::oo  oo:::::::::::oo         tt:::::::::::tt  ee:::::::::::::e   r:::::r                 S:::::::::::::::SS   uu::::::::uu:::ub:::::::::::::::b   s:::::::::::ss       y:::::y         s:::::::::::ss          tt:::::::::::tt  ee:::::::::::::e  m::::m   m::::m   m::::m
 *     SSSSSSSSSSSSSSS    hhhhhhh     hhhhhhh   ooooooooooo      ooooooooooo             ttttttttttt      eeeeeeeeeeeeee   rrrrrrr                  SSSSSSSSSSSSSSS       uuuuuuuu  uuuubbbbbbbbbbbbbbbb     sssssssssss        y:::::y           sssssssssss              ttttttttttt      eeeeeeeeeeeeee  mmmmmm   mmmmmm   mmmmmm
 *                                                                                                                                                                                                                             y:::::y
 *                                                                                                                                                                                                                            y:::::y

 */

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;
import com.ctre.phoenix.motorcontrol.can.TalonFX;
import com.revrobotics.CANSparkMax;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.Constants.ShooterConstants;
import frc.robot.utility.LoggingUtil;
import frc.robot.utility.ShooterSpeeds;
import frc.robot.utility.SparkMaxUtility;
import frc.robot.utility.TalonFXUtility;

// import com.revrobotics.CANSparkMax.ControlType;
// import com.revrobotics.SparkMaxPIDController;

public class ShooterSubsystem extends SubsystemBase {

  /* Shooter Talon FX Definition */
  TalonFX feederMotor;
  TalonFX flyWheelMotor;
  
  CANSparkMax intakeMotor;
  ShooterSpeeds speeds;

  PIDController feederPID;

  NetworkTable table;

  int encoderTicks = 2048;

  // The below calculations convert generated values from sysID from revolutions to ticks
  SimpleMotorFeedforward feedforwardBottom = new SimpleMotorFeedforward(0.71363 / encoderTicks, 0.10695 / encoderTicks, 0.0046128 / encoderTicks);
  SimpleMotorFeedforward feedforwardTop = new SimpleMotorFeedforward(0.52303 / encoderTicks, 0.10904 / encoderTicks, 0.0041191 / encoderTicks);

  public ShooterSubsystem() {

    feederMotor = TalonFXUtility.constructTalonFX(Constants.RobotMap.SHOOTER_FEEDER_CAN);
    flyWheelMotor = TalonFXUtility.constructTalonFX(Constants.RobotMap.SHOOTER_FLYWHEEL_CAN);

    intakeMotor = SparkMaxUtility.constructSparkMax(Constants.RobotMap.INTAKE_CAN, true);

    feederPID = new PIDController(Constants.ShooterConstants.FLYWHEEL_KP, 0, 0);

    speeds = new ShooterSpeeds(15000, 15000);

    table = NetworkTableInstance.getDefault().getTable("Shooter");

    /* Set PID */
    flyWheelMotor.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0, 0);
    flyWheelMotor.config_kP(0, 0.085035, 0);
    flyWheelMotor.config_kI(0, .00035, 0);
    flyWheelMotor.config_kD(0, 0, 0);

    feederMotor.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0, 0);
    feederMotor.config_kP(0, 0.092851, 0);// .092851
    feederMotor.config_kI(0, .00035, 0);
    feederMotor.config_kD(0, 0, 0);

    LoggingUtil.logWithNetworkTable(table, "Is Shooter Ready", false);
    LoggingUtil.logWithNetworkTable(table, "Top Vel", 0);
    LoggingUtil.logWithNetworkTable(table, "Bot Vel", 0);
    LoggingUtil.logWithNetworkTable(table, "Top Setpt", 0);
    LoggingUtil.logWithNetworkTable(table, "Bot Setpt", 0);

  }

  public void toSetPoint(double setpoint) {
    feederMotor.set(TalonFXControlMode.Velocity, setpoint);
  }

  public void shootPID() {

    SmartDashboard.putNumber("speeds", speeds.getFeederVelocity());
    flyWheelMotor.config_kF(0, ShooterConstants.SHOOTER_KF, 0);
    feederMotor.config_kF(0, ShooterConstants.SHOOTER_KF, 0);

    SmartDashboard.putNumber("FF Constant", feedforwardBottom.calculate(speeds.getFeederVelocity()));

    flyWheelMotor.set(TalonFXControlMode.Velocity, speeds.getFlywheelVelocity());
    feederMotor.set(TalonFXControlMode.Velocity, speeds.getFeederVelocity());

  }

  public void stopMotors() {
    feederMotor.set(TalonFXControlMode.PercentOutput, 0); // .9 Bottom
    flyWheelMotor.set(TalonFXControlMode.PercentOutput, 0); // Top
  }

  @Override
  public void periodic() {
    log();
  }

  public void spinIntake(double speed) {
    SparkMaxUtility.runSparkMax(intakeMotor, -speed);
  }

  public boolean checkAtSetpoint() {
    if((Math.abs(flyWheelMotor.getSelectedSensorVelocity() - speeds.getFlywheelVelocity()) < Constants.ShooterConstants.TARGET_THRESHOLD) && (Math.abs(feederMotor.getSelectedSensorVelocity() - speeds.getFeederVelocity()) < Constants.ShooterConstants.TARGET_THRESHOLD)) {
      return true;
    } else {
      return false;
    }
  }

  public void log() {
    LoggingUtil.logWithNetworkTable(table, "Is Shooter Ready", checkAtSetpoint());
    LoggingUtil.logWithNetworkTable(table, "Top Vel", flyWheelMotor.getSelectedSensorVelocity());
    LoggingUtil.logWithNetworkTable(table, "Bot Vel", feederMotor.getSelectedSensorVelocity());
    LoggingUtil.logWithNetworkTable(table, "Top Setpt", speeds.getFlywheelVelocity());
    LoggingUtil.logWithNetworkTable(table, "Bot Setpt", speeds.getFeederVelocity());
  }

}
