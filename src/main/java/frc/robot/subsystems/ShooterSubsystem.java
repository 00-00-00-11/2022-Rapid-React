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

import java.util.HashMap;

import com.revrobotics.CANSparkMax;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants.*;
import frc.robot.utility.LimelightUtility;
import frc.robot.utility.LoggingUtil;
import frc.robot.utility.SparkMaxUtility;
import frc.robot.vision.Limelight;

public class ShooterSubsystem extends SubsystemBase {

  CANSparkMax feederMotor; 
  CANSparkMax flyWheelMotor;
  CANSparkMax intakeMotor;
  Limelight limelight;

  HashMap <String, Double> setpoints = new HashMap<String, Double>();

  public ShooterSubsystem() {
    feederMotor = SparkMaxUtility.constructSparkMax(RobotMap.SHOOTER_FEEDER_CAN, true);
    flyWheelMotor = SparkMaxUtility.constructSparkMax(RobotMap.SHOOTER_FLYWHEEL_CAN, true);
    intakeMotor = SparkMaxUtility.constructSparkMax(RobotMap.INTAKE_CAN, true);
    limelight  = LimelightUtility.constructLimelight(ShooterConstants.SHOOTER_ANGLE, ShooterConstants.LIMELIGHT_HEIGHT, FieldConstants.HIGH_GOAL_HEIGHT, ShooterConstants.PIPELINE);

    setpoints.put("flyWheelSetpoint", 0.0);
    setpoints.put("feederSetpoint", 0.0);
  }

  public void spinShooter(double setpoint1, double setpoint2) {
    System.out.println(setpoint1);
    System.out.println(setpoint2);
  }

  @Override
  public void periodic() {
    limelight.update();
    spinShooter(getSetpoints().get("flyWheelSetpoint"), getSetpoints().get("feederSetpoint"));
    log();
  }

  public void spinIntake(double speed) {
    SparkMaxUtility.runSparkMax(intakeMotor, speed);
  }

  public void changeSetpoints(double setpoint1, double setpoint2) {
    setpoints.put("flyWheelSetpoint", setpoint1);
    setpoints.put("feederSetpoint", setpoint2);
  }

  public HashMap<String, Double> getSetpoints() {
    return setpoints;
  }

  public void calculateSetpoints() {
    double flyWheelSetpoint = limelight.getHorizontalOffset();
    double feederSetpoint = limelight.getVerticalOffset();
    changeSetpoints(flyWheelSetpoint, feederSetpoint);
  }

  public void log() {
    LoggingUtil.log("Shooter", "Flywheel Setpoint", getSetpoints().get("flyWheelSetpoint"));
    LoggingUtil.log("Shooter", "Feeder Setpoint", getSetpoints().get("feederSetpoint"));
  }

}