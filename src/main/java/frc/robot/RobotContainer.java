package frc.robot;

import edu.wpi.first.wpilibj2.command.Command;

import edu.wpi.first.wpilibj.controller.RamseteController;
import edu.wpi.first.wpilibj2.command.RamseteCommand;
import java.io.IOException;
import edu.wpi.first.wpilibj.trajectory.TrajectoryConfig;
import edu.wpi.first.wpilibj.Filesystem;
import java.nio.file.Path;

//temporary, for auto purposes
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;

import edu.wpi.first.wpilibj.trajectory.TrajectoryUtil;

public class RobotContainer {

  // private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();

  // private final ExampleCommand m_autoCommand = new ExampleCommand(m_exampleSubsystem);

  public final static DriveSubsystem m_driveSubsystem = new DriveSubsystem();

  public RobotContainer() {

    configureButtonBindings();
  }

  private void configureButtonBindings() {}

  public Command getAutonomousCommand() {
      // note: add this all to a subsystem later and reference drivebase motors and change when we have drivebase commands
      CANSparkMAX drivebaseMotor = new CANSparkMAX (0, CANSparkMaxLowLevel.MotorType.kBrushless);
      encoder = drivebaseMotor.getEncoder();
      encoder.setPosition(0.0); //Sets motor position to a floating point number
      if (encoder.getPosition()*6.0*Math.PI/18.0 < 6) //6.0 ft = wheel diameter, 18.0 ft = gearbox ratio
        drivebaseMotor.set(-0.2);

    /** 2021 Auto Code
    TrajectoryConfig config = new TrajectoryConfig(
      Units.feetToMeters(2), 
      Units.feetToMeters(2)
    );

    config.setKinematics(m_driveSubsystem.getKinematics());

 Trajectory pathWeaverTrajectory = new Trajectory();
    Path trajectoryPath = Filesystem.getDeployDirectory().toPath().resolve("output/slalom.wpilib.json");
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

    **/
  }
}
