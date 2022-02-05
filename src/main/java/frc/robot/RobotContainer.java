package frc.robot;

import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.*;
import frc.robot.subsystems.*;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.math.trajectory.Trajectory;

import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.RamseteCommand;

import java.io.IOException;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.math.trajectory.TrajectoryGenerator;
import edu.wpi.first.wpilibj.Filesystem;
import java.nio.file.Path;
import java.util.List;

import edu.wpi.first.math.trajectory.TrajectoryUtil;

public class RobotContainer {

  public static final DriveSubsystem m_driveSubsystem = new DriveSubsystem();
  public static final PS4Controller operatorGamepad = new PS4Controller(0);

  public static final ExitTarmac m_exitTarmac = new ExitTarmac();
  public RobotContainer() {
    m_driveSubsystem.setDefaultCommand(new SimDrive());
    configureButtonBindings();

  }

  private void configureButtonBindings() {
    // for (int i = 0; i < 360; i += 45) {
    //   new POVButton(operatorGamepad, i).whileHeld(new QuickTurn(i));
    // }
  }

  //2021 Auto Code
  public Command getAutonomousCommand() {
    //return m_exitTarmac;
    TrajectoryConfig config = new TrajectoryConfig(
    Units.feetToMeters(2), 
    Units.feetToMeters(2)
  );

  config.setKinematics(RobotContainer.m_driveSubsystem.getKinematics());
/*
Trajectory pathWeaverTrajectory = new Trajectory();
  Path trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve("paths/test.wpilib.json");
  try {
    pathWeaverTrajectory = TrajectoryUtil.fromPathweaverJson(trajectoryPath);
  } catch (IOException e) {
    e.printStackTrace();
  }
  */
  Trajectory exampleTrajectory =
  TrajectoryGenerator.generateTrajectory(
      // Start at the origin facing the +X direction
      new Pose2d(0, 0, new Rotation2d(0)),
      // Pass through these two interior waypoints, making an 's' curve path
      List.of(new Translation2d(1, 1), new Translation2d(2, -1)),
      // End 3 meters straight ahead of where we started, facing forward
      new Pose2d(3, 0, new Rotation2d(0)),
      // Pass config
      config);

  RamseteCommand command = new RamseteCommand(
      exampleTrajectory,
      RobotContainer.m_driveSubsystem::getPose,
      new RamseteController(2.0, 0.7),
      RobotContainer.m_driveSubsystem.getFeedforward(),
      RobotContainer.m_driveSubsystem.getKinematics(),
      RobotContainer.m_driveSubsystem::getSpeeds,
      RobotContainer.m_driveSubsystem.getLeftPIDController(),
      RobotContainer.m_driveSubsystem.getRightPIDController(),
      RobotContainer.m_driveSubsystem::setOutput,
      RobotContainer.m_driveSubsystem 
  );

// Reset odometry to the starting pose of the trajectory.
RobotContainer.m_driveSubsystem.resetOdometry(exampleTrajectory.getInitialPose());

// Run path following command, then stop at the end.
return command.andThen(() -> RobotContainer.m_driveSubsystem.setOutput(0, 0));  
}
}
