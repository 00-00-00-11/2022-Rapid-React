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
  ShuffleboardTab intakeTab;
  NetworkTableEntry leftSolenoidState;
  NetworkTableEntry rightSolenoidState;

  /** Creates a new Intake. */
  public Intake() {
    leftSolenoid =
        new DoubleSolenoid(
            Constants.IntakeConstants.pchPort,
            PneumaticsModuleType.REVPH,
            Constants.IntakeConstants.leftSolenoidPortForward,
            Constants.IntakeConstants.leftSolenoidPortReverse);
    rightSolenoid =
        new DoubleSolenoid(
            Constants.IntakeConstants.pchPort,
            PneumaticsModuleType.REVPH,
            Constants.IntakeConstants.rightSolenoidPortForward,
            Constants.IntakeConstants.rightSolenoidPortReverse);
    intakeMotor =
        new CANSparkMax(
            Constants.RobotMap.INTAKE_CAN, CANSparkMaxLowLevel.MotorType.kBrushless);

    leftSolenoid.set(Value.kOff);
    rightSolenoid.set(Value.kOff);

    intakeTab = Shuffleboard.getTab("Intake");
    leftSolenoidState = intakeTab.add("Left Solenoid", "off").getEntry();
    rightSolenoidState = intakeTab.add("Right Solenoid", "off").getEntry();
  }

  public void forwardIntake() {
    leftSolenoid.set(Value.kForward);
    rightSolenoid.set(Value.kForward);
    leftSolenoidState.setValue("Extended");
    rightSolenoidState.setValue("Extended");
  }

  public void reverseIntake() {
    leftSolenoid.set(Value.kReverse);
    rightSolenoid.set(Value.kReverse);
    leftSolenoidState.setValue("Retracted");
    rightSolenoidState.setValue("Retracted");
  }

  public void spinIntake() {
    intakeMotor.set(Constants.IntakeConstants.intakeSpeed);
    System.out.println("SPIIIIIIIIIN");
  }

  public void stop() {
    intakeMotor.set(0);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }
}
