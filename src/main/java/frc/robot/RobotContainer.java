package frc.robot;

import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.ShootBall;
import frc.robot.subsystems.ShooterSubsystem;
import edu.wpi.first.wpilibj.Joystick;

public class RobotContainer {

  // private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();

  // private final ExampleCommand m_autoCommand = new ExampleCommand(m_exampleSubsystem);

  public final static ShooterSubsystem m_shooter_subsystem = new ShooterSubsystem();
  
  public RobotContainer() {

    configureButtonBindings();
  }
  //* Defines the ps4Controller and defines the shootButton as R2 on the ps4Controller *//
  public static Joystick ps4Controller = new Joystick(1);
  Joystickbutton shootButton = new Joystickbutton(ps4Controller, 8); 
  private void configureButtonBindings() {
    shootButton.whileHeld(new ShootBall());
  }

  public Command getAutonomousCommand() {
    return null;
    // return m_autoCommand;
  }
}
