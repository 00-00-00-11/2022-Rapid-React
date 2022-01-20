package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj.GenericHID;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.button.*;
import edu.wpi.first.wpilibj.util.Units;
import edu.wpi.first.wpilibj.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.trajectory.Trajectory;
import java.util.Arrays;
import edu.wpi.first.wpilibj.geometry.Rotation2d;
import edu.wpi.first.wpilibj.geometry.Pose2d;
import edu.wpi.first.wpilibj.controller.RamseteController;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import java.io.IOException;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;

import edu.wpi.first.wpilibj.Filesystem;
import java.nio.file.Path;
import edu.wpi.first.wpilibj.trajectory.TrajectoryUtil;

import frc.robot.commands.*;
import frc.robot.subsystems.*;
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
