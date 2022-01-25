// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class Intake extends SubsystemBase {
  private DoubleSolenoid leftSolenoid;
  private DoubleSolenoid rightSolenoid;
  private CANSparkMax intakeMotor;
  ShuffleboardTab tab;
  NetworkTableEntry leftSolenoidState;
  NetworkTableEntry rightSolenoidState;

  /** Creates a new Intake. */
  public Intake() {
    leftSolenoid =
        new DoubleSolenoid(
            PneumaticsModuleType.REVPH,
            Constants.IntakeConstants.leftSolenoidPortForward,
            Constants.IntakeConstants.leftSolenoidPortReverse);
    rightSolenoid =
        new DoubleSolenoid(
            PneumaticsModuleType.REVPH,
            Constants.IntakeConstants.rightSolenoidPortForward,
            Constants.IntakeConstants.rightSolenoidPortReverse);
    intakeMotor =
        new CANSparkMax(
            Constants.IntakeConstants.motorID, CANSparkMaxLowLevel.MotorType.kBrushless);
    leftSolenoid.set(Value.kReverse);
    rightSolenoid.set(Value.kReverse);
    ShuffleboardTab tab = Shuffleboard.getTab("Intake");
    leftSolenoidState = tab.add("Left Solenoid", leftSolenoid.get()).getEntry();
    rightSolenoidState = tab.add("Right Solenoid", rightSolenoid.get()).getEntry();
  }

  public void toggleIntake() {
    leftSolenoid.toggle();
    rightSolenoid.toggle();
    leftSolenoidState.setValue(leftSolenoid.get());
    rightSolenoidState.setValue(rightSolenoid.get());
  }

  public void spinIntake() {
    intakeMotor.set(Constants.IntakeConstants.intakeSpeed);
  }

  public void stop() {
    intakeMotor.set(0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
