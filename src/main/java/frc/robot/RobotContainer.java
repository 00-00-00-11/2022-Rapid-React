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

  public RobotContainer() {
    m_driveSubsystem.setDefaultCommand(new SimDrive());
    configureButtonBindings();

    for (File jsonFile :
        Filesystem.getDeployDirectory().toPath().resolve("output").toFile().listFiles()) {
      String name = jsonFile.getName().replaceAll(".wpilib.json", "");

      Trajectory pathWeaverTrajectory = new Trajectory();
      Path trajectoryPath = jsonFile.toPath();
      try {
        pathWeaverTrajectory = TrajectoryUtil.fromPathweaverJson(trajectoryPath);
      } catch (IOException e) {
        e.printStackTrace();
      }

      m_chooser.addOption(name, pathWeaverTrajectory);
    }
    m_chooser.setDefaultOption(
        "nothing",
        TrajectoryUtil.deserializeTrajectory(
            "[{\"acceleration\": 0.0,\"curvature\": 0.0,\"pose\": {\"rotation\": {\"radians\": 0.0},\"translation\": {\"x\": 0.0,\"y\": 0.0}},\"time\": 0.0,\"velocity\": 0.0}]"));

    SmartDashboard.putData(m_chooser);
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
    // TrajectoryConfig config = new TrajectoryConfig(.5, .5);
    // ArrayList<Pose2d> states = new ArrayList<>();
    // for (State state : trajectory.getStates()) {
    //   states.add(state.poseMeters);
    // }
    // trajectory = TrajectoryGenerator.generateTrajectory(states, config);

    RamseteCommand command =
        new RamseteCommand(
            trajectory,
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
    m_driveSubsystem.resetOdometry(trajectory.getInitialPose());
    m_driveSubsystem.field.getObject("traj").setTrajectory(trajectory);

    // Run path following command, then stop at the end.
    return command.andThen(() -> RobotContainer.m_driveSubsystem.setOutput(0, 0));
  }
}
