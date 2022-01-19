package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.subsystems.*;
import frc.robot.commands.*;
import edu.wpi.first.wpilibj.PS4Controller;

public class RobotContainer {
  public static PS4Controller psController=new PS4Controller(Constants.IntakeConstants.ps4Port);
  public static final JoystickButton extendIntakeButton= new JoystickButton(psController, Constants.IntakeConstants.square);
  

  // private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();
  public static final Intake m_intake = new Intake();
  // private final ExampleCommand m_autoCommand = new ExampleCommand(m_exampleSubsystem);

  public RobotContainer() {

    configureButtonBindings();
  }

  private void configureButtonBindings() {
    extendIntakeButton.whenPressed(new IntakeExtend());
  }

  public Command getAutonomousCommand() {
    return null;
    // return m_autoCommand;
  }
}
