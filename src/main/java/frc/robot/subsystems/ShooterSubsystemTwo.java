// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.ctre.phoenix.motorcontrol.can.TalonFX;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.utility.TalonFXUtility;

public class ShooterSubsystemTwo extends SubsystemBase {
  /** Creates a new ShooterSubsystemTwo. */

  TalonFX feederMotor = TalonFXUtility.constructTalonFX(Constants.RobotMap.SHOOTER_FEEDER_CAN);
  TalonFX flyWheelMotor = TalonFXUtility.constructTalonFX(Constants.RobotMap.SHOOTER_FLYWHEEL_CAN);

  public ShooterSubsystemTwo() {}

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
