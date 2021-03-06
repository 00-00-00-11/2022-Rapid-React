// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.ShooterCommands;

import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;

/** An example command that uses an example subsystem. */
public class ShootBall extends CommandBase {

    @SuppressWarnings({ "PMD.UnusedPrivateField", "PMD.SingularField" })
    /**
     * Creates a new ShootCommand.
     *
     * @param subsystem The subsystem used by this command.
     */
    public ShootBall() {
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(RobotContainer.m_shooterSubsystem);
    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {}

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        RobotContainer.m_shooterSubsystem.shootPID();
        // RobotContainer.m_shooterSubsystem.shootBall();
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        if (interrupted) {
            RobotContainer.m_shooterSubsystem.stopMotors();
        }
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}
