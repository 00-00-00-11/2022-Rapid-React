// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.PneumaticsModuleType;


import frc.robot.Constants;

public class Intake extends SubsystemBase {
  private DoubleSolenoid leftSolenoid;
  private DoubleSolenoid rightSolenoid;

  /** Creates a new Intake. */
  public Intake() {
  leftSolenoid= new DoubleSolenoid(PneumaticsModuleType.CTREPCM, Constants.IntakeConstants.leftSolenoidPortForward, Constants.IntakeConstants.leftSolenoidPortReverse);
  rightSolenoid= new DoubleSolenoid(PneumaticsModuleType.CTREPCM, Constants.IntakeConstants.rightSolenoidPortForward, Constants.IntakeConstants.rightSolenoidPortReverse);
  leftSolenoid.set(Value.kReverse);
  rightSolenoid.set(Value.kReverse);
  }

  public void toggleIntake() {
    leftSolenoid.toggle();
    rightSolenoid.toggle();
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
