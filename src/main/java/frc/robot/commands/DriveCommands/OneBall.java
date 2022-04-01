// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.commands.DriveCommands;

import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;

public class OneBall extends SequentialCommandGroup {

    /** Creates a new OneBall. */
    public OneBall() {
        addCommands(
            new DriveToDistance(2.0)
        );
    }
}
