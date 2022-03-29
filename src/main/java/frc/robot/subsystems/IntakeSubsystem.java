// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.networktables.NetworkTableEntry;
import edu.wpi.first.wpilibj.DoubleSolenoid;
import edu.wpi.first.wpilibj.DoubleSolenoid.Value;
import edu.wpi.first.wpilibj.PneumaticsModuleType;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import edu.wpi.first.wpilibj.shuffleboard.ShuffleboardTab;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class IntakeSubsystem extends SubsystemBase {
   private DoubleSolenoid leftSolenoid;
   private DoubleSolenoid rightSolenoid;
  ShuffleboardTab intakeTab;
  NetworkTableEntry leftSolenoidState;
  NetworkTableEntry rightSolenoidState;

  public IntakeSubsystem() {
    leftSolenoid =
        new DoubleSolenoid(
            Constants.RobotMap.HUB_CAN,
            PneumaticsModuleType.REVPH,
            Constants.RobotMap.HUB_SOLENOID1_2,
            Constants.RobotMap.HUB_SOLENOID1_1);

    rightSolenoid =
        new DoubleSolenoid(
            Constants.RobotMap.HUB_CAN,
            PneumaticsModuleType.REVPH,
            Constants.RobotMap.HUB_SOLENOID2_2,
            Constants.RobotMap.HUB_SOLENOID2_1);

    leftSolenoid.set(Value.kReverse);
    rightSolenoid.set(Value.kReverse);

    intakeTab = Shuffleboard.getTab("Dashboard");
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

}
