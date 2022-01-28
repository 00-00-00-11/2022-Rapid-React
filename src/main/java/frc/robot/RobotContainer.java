package frc.robot;

import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.*;
import frc.robot.subsystems.*;

public class RobotContainer {

  public static final DriveSubsystem m_driveSubsystem = new DriveSubsystem();
  public static final PS4Controller operatorGamepad = new PS4Controller(0);

  public RobotContainer() {
    m_driveSubsystem.setDefaultCommand(new SimDrive());
    configureButtonBindings();
  }

  private void configureButtonBindings() {}

  public Command getAutonomousCommand() {
    return null;
    // return m_autoCommand;
  }
}
