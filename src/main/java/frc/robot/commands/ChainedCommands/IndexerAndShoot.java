// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.ChainedCommands;

import edu.wpi.first.wpilibj2.command.ParallelCommandGroup;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.WaitCommand;
import frc.robot.commands.IndexerCommands.RunIndexerWithoutProximity;
import frc.robot.commands.ShooterCommands.ShootBall;

// NOTE:  Consider using this command inline, rather than writing a subclass.  For more
// information, see:
// https://docs.wpilib.org/en/stable/docs/software/commandbased/convenience-features.html
public class IndexerAndShoot extends ParallelCommandGroup {

    /** Creates a new IndexerAndShoot. */
    public IndexerAndShoot() {
        addCommands(new ShootBall(), new SequentialCommandGroup(new WaitCommand(1), new RunIndexerWithoutProximity())); // origional
                                                                                                                        // 1.25

    }
}
