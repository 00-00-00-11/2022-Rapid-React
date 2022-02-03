package frc.robot;

import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj2.command.Command;
import frc.robot.commands.*;
import frc.robot.subsystems.*;

import edu.wpi.first.math.util.Units;
import edu.wpi.first.math.trajectory.Trajectory;

import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.wpilibj2.command.RamseteCommand;

import java.io.IOException;
import edu.wpi.first.math.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.Filesystem;
import java.nio.file.Path;
import edu.wpi.first.math.trajectory.TrajectoryUtil;

public class RobotContainer {

  public static final DriveSubsystem m_driveSubsystem = new DriveSubsystem();
  public static final PS4Controller operatorGamepad = new PS4Controller(0);

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
  
    TrajectoryConfig config = new TrajectoryConfig(
      Units.feetToMeters(2), 
      Units.feetToMeters(2)
    );

    config.setKinematics(m_driveSubsystem.getKinematics());

 Trajectory pathWeaverTrajectory = new Trajectory();
    Path trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve("paths/test.wpilib.json");
    try {
      pathWeaverTrajectory = TrajectoryUtil.fromPathweaverJson(trajectoryPath);
    } catch (IOException e) {
      e.printStackTrace();
    }

    RamseteCommand command = new RamseteCommand(
        pathWeaverTrajectory,
        m_driveSubsystem::getPose,
        new RamseteController(2.0, 0.7),
        m_driveSubsystem.getFeedforward(),
        m_driveSubsystem.getKinematics(),
        m_driveSubsystem::getSpeeds,
        m_driveSubsystem.getLeftPIDController(),
        m_driveSubsystem.getRightPIDController(),
        m_driveSubsystem::setOutput,
        m_driveSubsystem 
    );

    // Reset odometry to the starting pose of the trajectory.
    m_driveSubsystem.resetOdometry(pathWeaverTrajectory.getInitialPose());

    // Run path following command, then stop at the end.
    return command.andThen(() -> m_driveSubsystem.setOutput(0, 0));
  }
}
