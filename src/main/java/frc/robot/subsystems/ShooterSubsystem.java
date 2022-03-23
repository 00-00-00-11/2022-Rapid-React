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

import frc.robot.utility.TalonFXUtility;
import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;


import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.utility.ShooterSpeeds;
import frc.robot.utility.SparkMaxUtility;
import com.revrobotics.CANSparkMax;

import java.util.TreeMap;

// import com.revrobotics.CANSparkMax.ControlType;
// import com.revrobotics.SparkMaxPIDController;

public class ShooterSubsystem extends SubsystemBase {

  /* Velocity Conversion */
  double velConv = 10.0 / 4096;

  /* Interpolating Table */


  /* Shooter Talon FX Definition */
  TalonFX feederMotor = TalonFXUtility.constructTalonFX(Constants.RobotMap.SHOOTER_FEEDER_CAN);
  TalonFX flyWheelMotor = TalonFXUtility.constructTalonFX(Constants.RobotMap.SHOOTER_FLYWHEEL_CAN);
  ShooterSpeeds speeds;

  CANSparkMax intakeMotor = SparkMaxUtility.constructSparkMax(Constants.RobotMap.INTAKE_CAN, true);

  public ShooterSubsystem() {
    speeds = new ShooterSpeeds(2000, 2000);

    /* Preferences */
    Preferences.setDouble("Feeder KP", Constants.ShooterConstants.FLYWHEEL_KP);

    /* Set PID */
    flyWheelMotor.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0, 0);

    flyWheelMotor.config_kP(0, 3000, 0);
    flyWheelMotor.config_kI(0, 0, 0);
    flyWheelMotor.config_kD(0, 0, 0);
    flyWheelMotor.config_kF(0, 0, 0);

  }

  public void toSetPoint(double setpoint) {
    feederMotor.set(TalonFXControlMode.Velocity, setpoint);
  }

  public void shootPID() { 
     // feederMotor.set(TalonFXControlMode.Velocity,12.880859/velConv); // BOTTOM
     feederMotor.set(TalonFXControlMode.PercentOutput,1); //.9
     flyWheelMotor.set(TalonFXControlMode.PercentOutput,1); //TOP
  }
  public void stopMotors() { 
    // feederMotor.set(TalonFXControlMode.Velocity,12.880859/velConv); // BOTTOM
    feederMotor.set(TalonFXControlMode.PercentOutput,0); //.9
     flyWheelMotor.set(TalonFXControlMode.PercentOutput,0); //TOP
 }



  @Override
  public void periodic() {
    speeds.setFeederVelocity(200000);
    speeds.setFlywheelVelocity(200000);      
    SmartDashboard.putNumber("TOP Shooter Vel", flyWheelMotor.getSelectedSensorVelocity() * velConv);
    SmartDashboard.putNumber("BOTTOM Shooter Vel", feederMotor.getSelectedSensorVelocity() * velConv);
    log();

  }

  public void spinIntake(double speed) {
    SparkMaxUtility.runSparkMax(intakeMotor, -speed);
  }

  public void changeSetpoints(double setpoint1, double setpoint2) {
    speeds.setFlywheelVelocity(setpoint1);
    speeds.setFeederVelocity(setpoint2);
  }

  public void log() {
    SmartDashboard.putNumber("TOP Shooter Vel", flyWheelMotor.getSelectedSensorVelocity() * velConv);
    SmartDashboard.putNumber("BOTTOM Shooter Vel", feederMotor.getSelectedSensorVelocity() * velConv);
    SmartDashboard.putNumber("TOP Shooter Setpoint", speeds.getFlywheelVelocity());
    SmartDashboard.putNumber("BOTTOM Shooter Setpoint", speeds.getFeederVelocity());
  }

}