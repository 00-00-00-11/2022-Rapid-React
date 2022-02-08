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
 *                                                                                                                                                                                                                           y:::::y
 *                                                                                                                                                                                                                          y:::::y
 *                                                                                                                                                                                                                         yyyyyyy
 *
 *
 */

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.SparkMaxPIDController;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.utility.SparkMaxUtility;

public class ShooterSubsystem extends SubsystemBase {
  /* Shooter CANSpark Definition */
  CANSparkMax feederMotor =
      SparkMaxUtility.constructSparkMax(Constants.ShooterConstants.FEEDER_MOTOR_PORT, true);
  CANSparkMax flyWheelMotor =
      SparkMaxUtility.constructSparkMax(Constants.ShooterConstants.FLY_WHEEL_MOTOR_PORT, true);

  /* Shooter CANPIDController Definition */
  private SparkMaxPIDController feederPIDController = feederMotor.getPIDController();
  private SparkMaxPIDController flyWheelPIDController = flyWheelMotor.getPIDController();

  public ShooterSubsystem() {
    /* Feeder PID Controller */
    feederPIDController.setP(Constants.ShooterConstants.kP);
    feederPIDController.setI(Constants.ShooterConstants.kI);
    feederPIDController.setD(Constants.ShooterConstants.kD);
    feederPIDController.setIZone(Constants.ShooterConstants.kIz);
    feederPIDController.setFF(Constants.ShooterConstants.kFF);
    feederPIDController.setOutputRange(
        Constants.ShooterConstants.kMinOutput, Constants.ShooterConstants.kMaxOutput);

    /* Fly Wheel PID Controller */
    flyWheelPIDController.setP(Constants.ShooterConstants.kP);
    flyWheelPIDController.setI(Constants.ShooterConstants.kI);
    flyWheelPIDController.setD(Constants.ShooterConstants.kD);
    flyWheelPIDController.setIZone(Constants.ShooterConstants.kIz);
    flyWheelPIDController.setFF(Constants.ShooterConstants.kFF);
    flyWheelPIDController.setOutputRange(
        Constants.ShooterConstants.kMinOutput, Constants.ShooterConstants.kMaxOutput);
  }
  /* Shoots The Ball */
  public void shootBalls(boolean shoot) {
    if (shoot) {
      // Sets The Speeds From PID Constants
      double speed = Constants.ShooterConstants.multiplier * Constants.ShooterConstants.maxRPM;

      // Spins The Feeder And Fly Wheel Motor
      feederPIDController.setReference(speed, ControlType.kVelocity);
      flyWheelPIDController.setReference(speed, ControlType.kVelocity);
    } else {
      feederMotor.set(0);
      flyWheelMotor.set(0);
    }
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }

  public void shootCIM(double speed) {
    feederMotor.set(speed);
    flyWheelMotor.set(speed);
  }
}
