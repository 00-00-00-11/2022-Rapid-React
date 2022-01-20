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

  DigitalInput anglerMaxSwitch;
  DigitalInput anglerMinSwitch;

  public ClimberSubsystem() {
    primaryElevatorMotor =
        new CANSparkMax(
            Constants.ElevatorConstants.ELEVATOR_MOTOR, CANSparkMaxLowLevel.MotorType.kBrushless);
    secondaryAnglerMotor =
        new CANSparkMax(
            Constants.ElevatorConstants.ANGLER_MOTOR, CANSparkMaxLowLevel.MotorType.kBrushless);

    elevatorMaxSwitch = new DigitalInput(Constants.ElevatorConstants.ELEVATOR_MAX_SWITCH);
    elevatorMinSwitch = new DigitalInput(Constants.ElevatorConstants.ELEVATOR_MIN_SWITCH);

    anglerMaxSwitch = new DigitalInput(Constants.ElevatorConstants.ANGLER_MAX_SWITCH);
    anglerMinSwitch = new DigitalInput(Constants.ElevatorConstants.ANGLER_MIN_SWITCH);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  // button hit by driver, elevator height to max
  public void elevatorUpDriver() {
    // Max height implementation
    if (elevatorMaxSwitch.get() || elevatorMinSwitch.get()) {
      // stop elevator
    } else {
      // run elevator up
    }
  }

  public void elevatorUpDriver(double distance) {
    if (elevatorMaxSwitch.get() || elevatorMinSwitch.get()) {
      // stop elevator
    } else {
      // run elevator to set height (PID?)
    }
  }

  public void resetElevator() {
    if (elevatorMinSwitch.get()) {
      primaryElevatorMotor.getEncoder().setPosition(0);
    } else {
      primaryElevatorMotor.set(0.05); // change to whatever direction
    }
  }

  public void resetAngler() {
    if (anglerMinSwitch.get()) {
      secondaryAnglerMotor.getEncoder().setPosition(0);
    } else {
      secondaryAnglerMotor.set(0.05); // change to whatever direction
    }
  }

  public void resetAll() {
    resetAngler();
    resetElevator();
  }

  public void nextRung() {
    // Run autonomous commands here

    /*
      1. primary extends small distance
      2. secondary extends small angle
      3. primary retracts
      4. secondary returns to 0 angle
      5. primary extends partially
      6. secondary rotates large angle
      7. primary extends max
      8. secondary rotates till primary impacts
      9. primary retracts min
      10. secondary returns to 0 angle
    */
  }

  public boolean climber01() {
    /*
      1. primary extends small distance
      2. secondary extends small angle
    */
    boolean elevatorDone = false;
    boolean anglerDone = false;

    if (primaryElevatorMotor.getEncoder().getPosition()
        > Constants.ElevatorConstants.ELEVATOR_SMALL_DISTANCE) {
      primaryElevatorMotor.set(0.1);
    } else {
      primaryElevatorMotor.set(0);
      elevatorDone = true;
    }

    if (secondaryAnglerMotor.getEncoder().getPosition()
        < Constants.ElevatorConstants.ANGLER_SMALL_ANGLE) {
      secondaryAnglerMotor.set(0.1);
    } else {
      secondaryAnglerMotor.set(0);
      anglerDone = true;
    }

    return anglerDone && elevatorDone;
  }

  public boolean climber02() {
    /*
      3. primary retracts
      4. secondary returns to 0 angle
    */

    boolean elevatorDone = false;
    boolean anglerDone = false;

    if (!elevatorMinSwitch.get()) {
      primaryElevatorMotor.set(-0.1);
    } else {
      primaryElevatorMotor.set(0);
      elevatorDone = true;
    }

    if (!anglerMinSwitch.get()) {
      secondaryAnglerMotor.set(-0.1);
    } else {
      secondaryAnglerMotor.set(0);
      anglerDone = true;
    }

    return anglerDone && elevatorDone;
  }

  public boolean climber03() {
    /*
      5. primary extends partially
    */

    boolean elevatorDone = false;

    if (primaryElevatorMotor.getEncoder().getPosition() > Constants.ElevatorConstants.ELEVATOR_SMALL_DISTANCE) {
      primaryElevatorMotor.set(0.1);
    } else {
      primaryElevatorMotor.set(0);
      elevatorDone = true;
    }

    return elevatorDone;
  }

  public boolean climber04() {
    /*
      6. secondary rotates large angle
    */

    return true;
  }

  public boolean climber05() {
    /*
      7. primary extends max
    */

    return true;
  }

  public boolean climber06() {
    /*
      8. secondary rotates till primary impacts
    */

    return true;
  }

  public boolean climber07() {
    /*
      9. primary retracts min
      10. secondary returns to 0 angle
    */
    return true;
  }
}
