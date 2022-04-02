// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.DriveCommands;

import frc.robot.RobotContainer;
import frc.robot.commands.ChainedCommands.IndexerAndShoot;
import frc.robot.commands.ChainedCommands.IntakeAndIndex;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.ParallelRaceGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;

public class TwoBall2 extends SequentialCommandGroup {
  /** Creates a new TwoBall2. */
  public TwoBall2() {
    addCommands(
        new InstantCommand(
            () -> {
              RobotContainer.m_intakeSubsystem.forwardIntake();
            }),

        new ParallelCommandGroup(
            new DriveToDistance(0.5),
            new IntakeAndIndex()),

        new TurnDegrees(180),

        new DriveToDistance(0.5),

        new ParallelRaceGroup(new IndexerAndShoot(), new WaitCommand(5.0)),

        new AutoMoveBack());

  }
}
