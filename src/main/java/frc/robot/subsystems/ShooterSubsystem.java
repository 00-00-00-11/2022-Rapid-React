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

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.wpilibj.Preferences;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

import com.ctre.phoenix.motorcontrol.TalonFXControlMode;
import com.ctre.phoenix.motorcontrol.TalonFXFeedbackDevice;


import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.utility.ShooterSpeeds;
import frc.robot.utility.SparkMaxUtility;
import com.revrobotics.CANSparkMax;

// import com.revrobotics.CANSparkMax.ControlType;
// import com.revrobotics.SparkMaxPIDController;

public class ShooterSubsystem extends SubsystemBase {

  /* Velocity Conversion */
  double velConv = 10.0 / 2048;

  /* Interpolating Table */


  /* Shooter Talon FX Definition */
  TalonFX feederMotor = TalonFXUtility.constructTalonFX(Constants.RobotMap.SHOOTER_FEEDER_CAN);
  TalonFX flyWheelMotor = TalonFXUtility.constructTalonFX(Constants.RobotMap.SHOOTER_FLYWHEEL_CAN);
  ShooterSpeeds speeds;

  PIDController feederPID = new PIDController(Constants.ShooterConstants.FLYWHEEL_KP, 0, 0);

  CANSparkMax intakeMotor = SparkMaxUtility.constructSparkMax(Constants.RobotMap.INTAKE_CAN, true);

  int val = 2048;

  SimpleMotorFeedforward feedforwardBottom = new SimpleMotorFeedforward(0.71363 / val,0.10695 / val,0.0046128 / val);
  SimpleMotorFeedforward feedforwardTop = new SimpleMotorFeedforward(0.52303 / val,0.10904 / val,0.0041191/ val);

  public ShooterSubsystem() 
  {
    speeds = new ShooterSpeeds(60, 60);

    /* Preferences */
    Preferences.setDouble("Feeder KP", Constants.ShooterConstants.FLYWHEEL_KP);

    /* Set PID */
    flyWheelMotor.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0, 0);
    //00025
    flyWheelMotor.config_kP(0, 0.085035, 0);
    flyWheelMotor.config_kI(0, .00035, 0);
    flyWheelMotor.config_kD(0, 0, 0);
 
    feederMotor.configSelectedFeedbackSensor(TalonFXFeedbackDevice.IntegratedSensor, 0, 0);

    feederMotor.config_kP(0, 0.092851, 0);//.092851
    feederMotor.config_kI(0, .00035, 0);
    feederMotor.config_kD(0, 0, 0);


  }

  public void toSetPoint(double setpoint) {
    feederMotor.set(TalonFXControlMode.Velocity, setpoint);
  }

  public void shootPID() {
    speeds = new ShooterSpeeds(15000, 15000);


    // double targetVelo=2048*Constants.ShooterConstants.targetVelocityRPM/600;
    // speeds.setFeederVelocity(targetVelo);
    // speeds.setFlywheelVelocity(targetVelo);

    SmartDashboard.putNumber("speeds", speeds.getFeederVelocity());
    flyWheelMotor.config_kF(0, 0.048973143759,0);
    feederMotor.config_kF(0, 0.048973143759, 0);
    // feederMotor.set(TalonFXControlMode.PercentOutput,.9); //.9 bottom

    SmartDashboard.putNumber("FF Constant", feedforwardBottom.calculate(speeds.getFeederVelocity()));

    //  flyWheelMotor.set(TalonFXControlMode.PercentOutput,1); //.4 top 
    // feederMotor.set(TalonFXControlMode.PercentOutput,1);
    
  //  feederMotor.set(TalonFXControlMode.Velocity, 60); //.9 Bottom
  //  flyWheelMotor.set(TalonFXControlMode.Velocity, 60); //Top

    feederMotor.set(TalonFXControlMode.Velocity, speeds.getFeederVelocity()); //.9 Bottom
   flyWheelMotor.set(TalonFXControlMode.Velocity, speeds.getFlywheelVelocity()); //Top
   
  }
  public void stopMotors() { 
    // feederMotor.set(TalonFXControlMode.Velocity,12.880859/velConv); // BOTTOM
    feederMotor.set(TalonFXControlMode.PercentOutput,0); //.9 Bottom
    flyWheelMotor.set(TalonFXControlMode.PercentOutput,0); //Top
 }



  @Override
  public void periodic() {
    // speeds.setFeederVelocity(204.8);
    // speeds.setFlywheelVelocity(204.8);      
    SmartDashboard.putNumber("TOP Shooter Velx", flyWheelMotor.getSelectedSensorVelocity());
    //SmartDashboard.putNumber("TOP Shooter Vel conversion", 23000/flyWheelMotor.getSelectedSensorVelocity() );

    SmartDashboard.putNumber("BOTTOM Shooter Velx", feederMotor.getSelectedSensorVelocity());
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
    SmartDashboard.putNumber("TOP Shooter Vel", flyWheelMotor.getSelectedSensorVelocity() );
    SmartDashboard.putNumber("BOTTOM Shooter Vel", feederMotor.getSelectedSensorVelocity() );
    SmartDashboard.putNumber("TOP Shooter Setpoint", speeds.getFlywheelVelocity());
    SmartDashboard.putNumber("BOTTOM Shooter Setpoint", speeds.getFeederVelocity());
  }

}