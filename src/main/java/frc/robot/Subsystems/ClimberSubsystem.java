// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

/*
How Elevator Should Work
Driver Does:
1. hook primary onto bar
2. retract primary to min


Automated (Run Once Per Button):
2. primary extends small distance
3. secondary extends small angle
3. primary retracts
4. secondary returns to 0 angle
5. primary extends partially
6. secondary rotates large angle
7. primary extends max
8. secondary rotates till primary impacts
9. primary retracts min
10. secondary returns to 0 angle
*/

package frc.robot.subsystems;

import com.revrobotics.*;
import com.revrobotics.CANSparkMax.IdleMode;

import edu.wpi.first.networktables.*;
import edu.wpi.first.wpilibj.PS4Controller;
import edu.wpi.first.wpilibj.shuffleboard.*;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ClimberSubsystem extends SubsystemBase {
  /** Creates a new ClimberSubsystem. */
  CANSparkMax elevatorMotor;
  CANSparkMax anglerMotorLeft;
  CANSparkMax anglerMotorRight;

  private ShuffleboardTab tab = Shuffleboard.getTab(Constants.ElevatorConstants.SHUFFLEBOARD_TAB);
  private NetworkTableEntry elevatorStepsEntry = tab.add("Elevator Step", "").getEntry();

  double ElevSpeed = Constants.ElevatorConstants.ELEVATOR_SPEED;
  
  double AnglerSpeed = Constants.ElevatorConstants.ANGLER_SPEED;

  int currentStep = 0;
  int lastFinishedStep = 0;

  public ClimberSubsystem() {
    elevatorMotor = new CANSparkMax(
        Constants.RobotMap.CLIMBER_LINEAR_CAN, 
        CANSparkMaxLowLevel.MotorType.kBrushless
      );

    anglerMotorLeft = new CANSparkMax(
        Constants.RobotMap.CLIMBER_LEFT_ARM_CAN, 
        CANSparkMaxLowLevel.MotorType.kBrushless
      );

    anglerMotorRight = new CANSparkMax(
      Constants.RobotMap.CLIMBER_RIGHT_ARM_CAN, 
      CANSparkMaxLowLevel.MotorType.kBrushless
    );

    elevatorMotor.setIdleMode(IdleMode.kBrake);
    anglerMotorLeft.setIdleMode(IdleMode.kBrake);
    anglerMotorRight.setIdleMode(IdleMode.kBrake);

    elevatorMotor.setInverted(false);
    anglerMotorLeft.setInverted(false);
    anglerMotorRight.setInverted(false);
  }

  @Override
  public void periodic() {}

  // Manual Control
  public void elevatorDriver(PS4Controller joystick) {
    System.out.println("manual control enabled");
    anglerMotorLeft.set(-0.1*(joystick.getR2Axis() - joystick.getL2Axis()));
    anglerMotorRight.set(0.1*(joystick.getR2Axis() - joystick.getL2Axis()));

    if (-joystick.getLeftY() < 0) {
      elevatorMotor.set(-0.5*joystick.getLeftY());
    } else {
      elevatorMotor.set(-0.5*joystick.getLeftY());
    }
  }
}

