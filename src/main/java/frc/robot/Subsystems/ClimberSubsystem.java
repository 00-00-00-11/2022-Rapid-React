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

package frc.robot.Subsystems;

import com.revrobotics.*;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ClimberSubsystem extends SubsystemBase {
  /** Creates a new ClimberSubsystem. */
  CANSparkMax primaryElevatorMotor;

  CANSparkMax secondaryAnglerMotor;

  DigitalInput elevatorMaxSwitch;
  DigitalInput elevatorMinSwitch;

  public ClimberSubsystem() {
    primaryElevatorMotor =
        new CANSparkMax(
            Constants.ElevatorConstants.ELEVATOR_MOTOR, CANSparkMaxLowLevel.MotorType.kBrushless);
    secondaryAnglerMotor =
        new CANSparkMax(
            Constants.ElevatorConstants.ANGLER_MOTOR, CANSparkMaxLowLevel.MotorType.kBrushless);

    elevatorMaxSwitch = new DigitalInput(Constants.ElevatorConstants.ELEVATOR_MAX_SWITCH);
    elevatorMinSwitch = new DigitalInput(Constants.ElevatorConstants.ELEVATOR_MIN_SWITCH);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  // button hit by driver, elevator height to max
  public void elevatorUpDriver(double distance) {
    // PID to set to specific height?
    // Or max height

    if (elevatorMaxSwitch.get() || elevatorMinSwitch.get()) {
      // stop elevator
    } else {
      // run elevator up
    }
  }
}
