// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.DriveCommands;

import frc.robot.RobotContainer;
import frc.robot.commands.ChainedCommands.IndexerAndShoot;
import frc.robot.commands.ChainedCommands.IntakeAndIndex;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class TwoBall extends SequentialCommandGroup {
  /** Creates a new TwoBall. */
  public TwoBall() {
    addCommands(

        new ExitTarmac(5),

        new ParallelRaceGroup(
            new IndexerAndShoot(),
            new WaitCommand(5.0)),

        new TurnDegrees(180),

        new InstantCommand(
            () -> {
              RobotContainer.m_intakeSubsystem.forwardIntake();
            }),

        new ParallelRaceGroup(
            new DriveToDistance(0.25),
            new IntakeAndIndex()),

        new TurnDegrees(180),

        new DriveToDistance(0.5),

        new ParallelRaceGroup(new IndexerAndShoot(), new WaitCommand(5.0)), // Waitcommand value arbitary

        new AutoMoveBack());
  }
}
