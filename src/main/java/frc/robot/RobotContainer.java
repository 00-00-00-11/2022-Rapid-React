package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;

public class RobotContainer {

  // private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();

  // private final ExampleCommand m_autoCommand = new ExampleCommand(m_exampleSubsystem);

  // started climber

  public RobotContainer() {

    configureButtonBindings();
  }

  private void configureButtonBindings() {}

  public Command getAutonomousCommand() {
    return null;
    // return m_autoCommand;
  }
}
