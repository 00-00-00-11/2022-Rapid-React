package frc.robot;

import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.RunIndexer;
import frc.robot.subsystems.IndexerSubsystem;

public class RobotContainer {
  public static IndexerSubsystem m_IndexerSubsystem = new IndexerSubsystem();
  public static PS4Controller operatorGamepad = new PS4Controller(0);
  public static JoystickButton indexerButton = new JoystickButton(operatorGamepad, 1);

  public RobotContainer() {
    configureButtonBindings();
  }

  private void configureButtonBindings() {
    indexerButton.whileHeld(new RunIndexer());
  }

  public Command getAutonomousCommand() {
    return null;
  }
}
