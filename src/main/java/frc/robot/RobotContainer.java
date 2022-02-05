package frc.robot;

import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.*;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.Commands.*;
import frc.robot.Subsystems.*;

public class RobotContainer {

  public static final ClimberSubsystem m_climberSubsystem = new ClimberSubsystem();

  public static final ClimberCommand m_climberCommand = new ClimberCommand(m_climberSubsystem);
  public static final AutoClimberCommand m_autoClimberCommand =
      new AutoClimberCommand(m_climberSubsystem);

  public static PS4Controller operatorJoystick =
      new PS4Controller(Constants.JoystickConstants.OPERATOR_JOYSTICK_PORT);

  Button driverElevatorButton =
      new JoystickButton(operatorJoystick, PS4Controller.Button.kCircle.value);
  Button autoElevatorButton =
      new JoystickButton(operatorJoystick, PS4Controller.Button.kCross.value);

  // started climber

  public RobotContainer() {
    configureButtonBindings();
  }

  private void configureButtonBindings() {
    autoElevatorButton.toggleWhenPressed(new AutoClimberCommand(m_climberSubsystem));
  }

  public Command getAutonomousCommand() {
    return null;
    // return m_autoCommand;
  }
}
