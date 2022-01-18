package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.Commands.ClimberCommand;
import frc.robot.Subsystems.ClimberSubsystem;

public class RobotContainer {

  // private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();

  private final ClimberSubsystem m_climberSubsystem = new ClimberSubsystem();

  // private final ExampleCommand m_autoCommand = new ExampleCommand(m_exampleSubsystem);

  private final ClimberCommand m_climberCommand = new ClimberCommand(m_climberSubsystem);

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
