// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.ClimbCommands;

import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj2.command.CommandBase;
import edu.wpi.first.wpilibj2.command.TrapezoidProfileCommand;
import frc.robot.RobotContainer;

public class ClimberToggle extends CommandBase {

    PS4Controller elevGamepad;

    /** Creates a new ManualClimber. */
    public ClimberToggle() {
        addRequirements(RobotContainer.m_climberSubsystem);
    }

    // Called when the command is initially scheduled.
    @Override

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        // RobotContainer.m_climberSubsystem.climberControl(elevGamepad);

    }

    alled once 
    he command 
    nds or is 

    rride

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}
