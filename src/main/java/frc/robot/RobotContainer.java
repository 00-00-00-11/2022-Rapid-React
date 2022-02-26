package frc.robot;

import edu.wpi.first.math.controller.RamseteController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.trajectory.Trajectory;
import edu.wpi.first.math.trajectory.TrajectoryUtil;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.JoystickButton;
import frc.robot.commands.*;
import frc.robot.subsystems.*;
import java.io.File;
import java.io.IOException;
import java.nio.file.Path;

public class RobotContainer {

  public static final PS4Controller operatorGamepad = new PS4Controller(0);

  public static final DriveSubsystem m_driveSubsystem = new DriveSubsystem();
  public static final ExitTarmac m_exitTarmac = new ExitTarmac();
  public static final ShooterSubsystem m_shooter_subsystem = new ShooterSubsystem();

  SendableChooser<Trajectory> m_chooser = new SendableChooser<>();
  Pose2d initialPose;
  
  Path straightPath;
  Path sinePath;
  Trajectory straight;
  Trajectory sine;
  
  public RobotContainer() {
    m_driveSubsystem.setDefaultCommand(new SimDrive());
    
    configureButtonBindings();
    
    straightPath = Filesystem.getDeployDirectory().toPath().resolve("output/straight7.wpilib.json");
    sinePath = Filesystem.getDeployDirectory().toPath().resolve("output/sine.wpilib.json");
  
    try {
      straight = TrajectoryUtil.fromPathweaverJson(straightPath);
      sine = TrajectoryUtil.fromPathweaverJson(sinePath);
    } catch (Exception e) {
      System.out.println("can't create trajectories");
    }

    SendableChooser mChooser = new SendableChooser<>();

    mChooser.addOption("4 Ball", null);
    mChooser.addOption("3 Ball", null);
    mChooser.addOption("2 Ball", null);
    mChooser.setDefaultOption("Exit Tarmac", new ExitTarmac());

    SmartDashboard.putData(mChooser);

  }

  // * Defines the ps4Controller and defines the shootButton as R2 on the
  // ps4Controller *//
  public static Joystick ps4Controller = new Joystick(1);
  JoystickButton shootButton = new JoystickButton(ps4Controller, 1);

  private void configureButtonBindings() {
    // for (int i = 0; i < 360; i += 45) {
    // new POVButton(operatorGamepad, i).whileHeld(new QuickTurn(i));
    // }
    shootButton.whileHeld(new ShootBall());
  }

  // 2021 Auto Code
  public Command getAutonomousCommand() {

    Trajectory trajectory = m_chooser.getSelected();

    RamseteCommand command =
        new RamseteCommand(
            straight,
            RobotContainer.m_driveSubsystem::getPose,
            new RamseteController(2, 0.7),
            RobotContainer.m_driveSubsystem.getFeedforward(),
            RobotContainer.m_driveSubsystem.getKinematics(),
            RobotContainer.m_driveSubsystem::getSpeeds,
            RobotContainer.m_driveSubsystem.getLeftPIDController(),
            RobotContainer.m_driveSubsystem.getRightPIDController(),
            RobotContainer.m_driveSubsystem::setOutput,
            RobotContainer.m_driveSubsystem);

    // Reset odometry to the starting pose of the trajectory.
    m_driveSubsystem.resetOdometry(straight.getInitialPose());
    // m_driveSubsystem.field.getObject("traj").setTrajectory(trajectory);

    RamseteCommand command2 =
        new RamseteCommand(
            sine,
            RobotContainer.m_driveSubsystem::getPose,
            new RamseteController(2, 0.7),
            RobotContainer.m_driveSubsystem.getFeedforward(),
            RobotContainer.m_driveSubsystem.getKinematics(),
            RobotContainer.m_driveSubsystem::getSpeeds,
            RobotContainer.m_driveSubsystem.getLeftPIDController(),
            RobotContainer.m_driveSubsystem.getRightPIDController(),
            RobotContainer.m_driveSubsystem::setOutput,
            RobotContainer.m_driveSubsystem);


    // Run path following command, then stop at the end.
    return (
      new SequentialCommandGroup(
        command.andThen(() -> RobotContainer.m_driveSubsystem.setOutput(0, 0)),
        command2.andThen(() -> RobotContainer.m_driveSubsystem.setOutput(0, 0))
      )
    );
  }
}
