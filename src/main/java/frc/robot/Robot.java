package frc.robot;

import com.revrobotics.CANSparkMax;
import com.revrobotics.REVPhysicsSim;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.Compressor;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.ScheduleCommand;

public class Robot extends TimedRobot {

    private Command m_autonomousCommand;

    private RobotContainer m_robotContainer;

    public static Compressor compressor;

    boolean finishedElevatorHoming = false;

    @Override
    public void robotInit() {
        compressor = new Compressor(Constants.RobotMap.HUB_CAN, PneumaticsModuleType.REVPH);
        try {
            m_robotContainer = new RobotContainer();
        } catch (Exception err) {
            err.printStackTrace();
            throw new ExceptionInInitializerError("[ERROR] COULDN'T INITIALIZE ROBOT CONTAINER");
        }

        SmartDashboard.putString("AUTO STATUS", "NO CHOSEN AUTO");
    }

    @Override
    public void robotPeriodic() {
        CommandScheduler.getInstance().run();
        SmartDashboard.putBoolean("Compressor Sensor", compressor.getPressureSwitchValue());
        if (compressor.getPressureSwitchValue()) {
            compressor.enableDigital();
        } else compressor.disable();
    }

    @Override
    public void disabledInit() {
        if (RobotContainer.m_driveSubsystem != null) {
            RobotContainer.m_driveSubsystem.setBrake(false);
        }
    }

    @Override
    public void disabledPeriodic() {}

    @Override
    public void autonomousInit() {
        RobotContainer.m_driveSubsystem.resetEncoders();
        if (RobotContainer.m_driveSubsystem != null) {
            RobotContainer.m_driveSubsystem.setBrake(true);
        }

        m_autonomousCommand = m_robotContainer.getAutonomousCommand();

        if (m_autonomousCommand != null) {
            m_autonomousCommand.schedule();
        }
    }

    /** This function is called periodically during autonomous. */
    @Override
    public void autonomousPeriodic() {}

    @Override
    public void teleopInit() {
        CommandScheduler.getInstance().cancelAll();
        RobotContainer.m_driveSubsystem.resetEncoders();

        if (m_autonomousCommand != null) {
            m_autonomousCommand.cancel();
        }

        finishedElevatorHoming = false;

        if (RobotContainer.m_driveSubsystem != null) {
            RobotContainer.m_driveSubsystem.setBrake(true);
        }
    }

    @Override
    public void teleopPeriodic() {
        if (!finishedElevatorHoming) {
            finishedElevatorHoming = RobotContainer.m_climberSubsystem.retractElevator();
        }
        
    }

    @Override
    public void testInit() {
        // Cancels all running commands at the start of test mode.
        CommandScheduler.getInstance().cancelAll();
    }

    /** This function is called periodically during test mode. */
    @Override
    public void testPeriodic() {}

    @Override
    public void simulationInit() {
        CANSparkMax[] motors = RobotContainer.m_driveSubsystem.getMotors();
        for (CANSparkMax motor : motors) {
            REVPhysicsSim.getInstance().addSparkMax(motor, DCMotor.getNEO(1));
        }
    }

    @Override
    public void simulationPeriodic() {
        // REVPhysicsSim.getInstance().run();

        // The above is not needed because we set velocity values manually in Drive Subsystem
    }
}
