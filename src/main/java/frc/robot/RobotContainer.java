package frc.robot;

import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.POVButton;
import frc.robot.commands.*;
import frc.robot.subsystems.*;

public class RobotContainer {

  public static final DriveSubsystem m_driveSubsystem = new DriveSubsystem();

  public static final PS4Controller operatorGamepad = new PS4Controller(0);

  public RobotContainer() {
    m_driveSubsystem.setDefaultCommand(new SimDrive());
    configureButtonBindings();
  }

  private void configureButtonBindings() {
    for (int i = 0; i < 360; i += 45) {
      new POVButton(operatorGamepad, i).whileHeld(new QuickTurn(i));
    }
  }

  public Command getAutonomousCommand() {
    return null;
    // return m_autoCommand;
  }
}
