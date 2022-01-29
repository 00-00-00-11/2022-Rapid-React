package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.*;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.*;
import frc.robot.subsystems.*;

public class RobotContainer {

  public static final PS4Controller operatorGamepad = new PS4Controller(0);

  public static final DriveSubsystem m_driveSubsystem = new DriveSubsystem();
  public static final ShooterSubsystem m_shooter_subsystem = new ShooterSubsystem();

  public RobotContainer() {
    m_driveSubsystem.setDefaultCommand(new SimDrive());
    configureButtonBindings();
  }
  // * Defines the ps4Controller and defines the shootButton as R2 on the ps4Controller *//
  public static Joystick ps4Controller = new Joystick(1);
  JoystickButton shootButton = new JoystickButton(ps4Controller, 8);

  private void configureButtonBindings() {
    shootButton.whileHeld(new ShootBall());
  }

  public Command getAutonomousCommand() {
    return null;
    // return m_autoCommand;
  }
}
