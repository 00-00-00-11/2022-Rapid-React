// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.IntakeCommands;

import edu.wpi.first.networktables.NetworkTable;
import edu.wpi.first.networktables.NetworkTableInstance;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.CommandBase;
import frc.robot.RobotContainer;

public class IntakeSpin extends CommandBase {
    NetworkTable table;
    boolean isExtended;

    /** Creates a new IntakeSpin. */
    public IntakeSpin() {
        
        // Use addRequirements() here to declare subsystem dependencies.
        addRequirements(RobotContainer.m_shooterSubsystem);
        

    }

    // Called when the command is initially scheduled.
    @Override
    public void initialize() {
        SmartDashboard.putString("Intake Status", "Up");
    }

    // Called every time the scheduler runs while the command is scheduled.
    @Override
    public void execute() {
        table = NetworkTableInstance.getDefault().getTable("Intake");
        isExtended = table.getEntry("Extended").getBoolean(true);
        if(isExtended){
            RobotContainer.m_shooterSubsystem.spinIntake(-0.5);

        }
    }

    // Called once the command ends or is interrupted.
    @Override
    public void end(boolean interrupted) {
        RobotContainer.m_shooterSubsystem.spinIntake(0);
    }

    // Returns true when the command should end.
    @Override
    public boolean isFinished() {
        return false;
    }
}
