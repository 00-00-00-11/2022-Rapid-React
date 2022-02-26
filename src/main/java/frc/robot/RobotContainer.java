package frc.robot;

import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.*;
import frc.robot.subsystems.*;
import frc.robot.utility.RamseteUtility;
import frc.robot.utility.TrajectoryUtility;
public class RobotContainer {

  public static final PS4Controller operatorGamepad = new PS4Controller(0);

  public static final DriveSubsystem m_driveSubsystem = new DriveSubsystem();
  public static final ShooterSubsystem m_shooter_subsystem = new ShooterSubsystem();
  
  public RobotContainer() {
    m_driveSubsystem.setDefaultCommand(new SimDrive());
    configureButtonBindings();
  }

  public static Joystick ps4Controller = new Joystick(1);
  JoystickButton shootButton = new JoystickButton(ps4Controller, 1);

  private void configureButtonBindings() {
    shootButton.whileHeld(new ShootBall());
  }

  public Command getAutonomousCommand() {
    System.out.println(m_driveSubsystem.getSelectedFromChooser());
    switch (m_driveSubsystem.getSelectedFromChooser()) {
      case 0:
        return new ExitTarmac();
      case 2:
        return m_driveSubsystem.TwoBallAuto(m_driveSubsystem);
      case 3:
        return m_driveSubsystem.ThreeBallAuto(m_driveSubsystem);
      case 4:
        return m_driveSubsystem.FourBallAuto(m_driveSubsystem);
      default:
        return null;
    }        
  }

}
