package frc.robot;

import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.*;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.*;
import frc.robot.subsystems.*;

public class RobotContainer {

  public static final ClimberSubsystem m_climberSubsystem = new ClimberSubsystem();

  public static final ClimberCommand m_climberCommand = new ClimberCommand(m_climberSubsystem);
  public static final AutoClimberCommand m_autoClimberCommand =
      new AutoClimberCommand(m_climberSubsystem);
  public static final ElevatorNextStep m_elevatorNextStep =
      new ElevatorNextStep(m_climberSubsystem);
  public static final ElevatorPrevStep m_elevatorPrevStep = new ElevatorPrevStep(m_climberSubsystem);

  public static Joystick operatorJoystick =
      new Joystick(Constants.JoystickConstants.OPERATOR_JOYSTICK_PORT);

  Button driverElevatorButton =
      new JoystickButton(operatorJoystick, Constants.JoystickConstants.DRIVER_ELEVATOR_BUTTON);
  Button autoElevatorButton =
      new JoystickButton(operatorJoystick, Constants.JoystickConstants.AUTO_ELEVATOR_BUTTON);

  public RobotContainer() {
    configureButtonBindings();
  }

  private void configureButtonBindings() {
    autoElevatorButton.whenPressed(new AutoClimberCommand(m_climberSubsystem));
    driverElevatorButton.whenPressed(new ElevatorNextStep(m_climberSubsystem));
  }

  public Command getAutonomousCommand() {
    return null;
    // return m_autoCommand;
  }
}
