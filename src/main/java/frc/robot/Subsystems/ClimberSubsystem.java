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
import edu.wpi.first.networktables.*;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.shuffleboard.*;
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

  private ShuffleboardTab tab = Shuffleboard.getTab(Constants.ElevatorConstants.SHUFFLEBOARD_TAB);
  private NetworkTableEntry elevatorStepsEntry = tab.add("Elevator Step", "").getEntry();

  int currentStep = 0;

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
      primaryElevatorMotor.set(
          Constants.ElevatorConstants.ELEVATOR_SPEED); // change to whatever direction
    }
  }

  public void resetAngler() {
    if (anglerMinSwitch.get()) {
      secondaryAnglerMotor.getEncoder().setPosition(0);
    } else {
      secondaryAnglerMotor.set(Constants.ElevatorConstants.ANGLER_SPEED);
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

    if (currentStep < 1) {
      climber001();
    } else if (currentStep < 2) {
      climber002();
    } else if (currentStep < 3) {
      climber003();
    } else if (currentStep < 4) {
      climber004();
    } else if (currentStep < 5) {
      climber005();
    } else if (currentStep < 6) {
      climber006();
    } else if (currentStep < 7) {
      climber007();
    } else {
      System.out.println("Climber Done");
      elevatorStepsEntry.setString("complete - ready for new loop");
    }
  }

  public boolean checkNextRungComplete() {
    return currentStep == 7;
  }

  public void resetSteps() {
    currentStep = 0;
  }

  public void climber001() {
    /*
      1. primary extends small distance
      2. secondary extends small angle
    */
    boolean elevatorDone = false;
    boolean anglerDone = false;

    elevatorStepsEntry.setString("step 1");

    if (primaryElevatorMotor.getEncoder().getPosition()
        > Constants.ElevatorConstants.ELEVATOR_SMALL_DISTANCE) {
      primaryElevatorMotor.set(Constants.ElevatorConstants.ELEVATOR_SPEED);
    } else {
      primaryElevatorMotor.set(0);
      elevatorDone = true;
    }

    if (secondaryAnglerMotor.getEncoder().getPosition()
        < Constants.ElevatorConstants.ANGLER_SMALL_ANGLE) {
      secondaryAnglerMotor.set(Constants.ElevatorConstants.ANGLER_SPEED);
    } else {
      secondaryAnglerMotor.set(0);
      anglerDone = true;
    }

    currentStep = (anglerDone && elevatorDone) ? (currentStep = 2) : (currentStep = 1);
  }

  public void climber002() {
    /*
      3. primary retracts
      4. secondary returns to 0 angle
    */

    boolean elevatorDone = false;
    boolean anglerDone = false;

    elevatorStepsEntry.setString("step 2");

    if (!elevatorMinSwitch.get()) {
      primaryElevatorMotor.set(-Constants.ElevatorConstants.ELEVATOR_SPEED);
    } else {
      primaryElevatorMotor.set(0);
      elevatorDone = true;
    }

    if (!anglerMinSwitch.get()) {
      secondaryAnglerMotor.set(-Constants.ElevatorConstants.ANGLER_SPEED);
    } else {
      secondaryAnglerMotor.set(0);
      anglerDone = true;
    }

    currentStep = (anglerDone && elevatorDone) ? (currentStep = 3) : (currentStep = 2);
  }

  public void climber003() {
    /*
      5. primary extends partially
    */

    boolean elevatorDone = false;

    elevatorStepsEntry.setString("step 3");

    if (primaryElevatorMotor.getEncoder().getPosition()
        > Constants.ElevatorConstants.ELEVATOR_SMALL_DISTANCE) {
      primaryElevatorMotor.set(Constants.ElevatorConstants.ELEVATOR_SPEED);
    } else {
      primaryElevatorMotor.set(0);
      elevatorDone = true;
    }

    currentStep = (elevatorDone) ? (currentStep = 4) : (currentStep = 3);
  }

  public void climber004() {
    /*
      6. secondary rotates large angle
    */

    boolean anglerDone = false;

    elevatorStepsEntry.setString("step 4");

    if (secondaryAnglerMotor.getEncoder().getPosition()
        < Constants.ElevatorConstants.ANGLER_LARGE_ANGLE) {
      secondaryAnglerMotor.set(Constants.ElevatorConstants.ANGLER_SPEED);
    } else {
      secondaryAnglerMotor.set(0);
      anglerDone = true;
    }

    currentStep = (anglerDone) ? (currentStep = 5) : (currentStep = 4);
  }

  public void climber005() {
    /*
      7. primary extends max
    */

    boolean elevatorDone = false;

    elevatorStepsEntry.setString("step 5");

    if (!elevatorMaxSwitch.get()) {
      primaryElevatorMotor.set(Constants.ElevatorConstants.ELEVATOR_SPEED);
    } else {
      primaryElevatorMotor.set(0);
      elevatorDone = true;
    }

    currentStep = (elevatorDone) ? (currentStep = 6) : (currentStep = 5);
  }

  public void climber006() {
    /*
      8. secondary rotates till primary impacts
    */

    boolean anglerDone = false;

    elevatorStepsEntry.setString("step 6");

    if (secondaryAnglerMotor.getEncoder().getPosition()
        > Constants.ElevatorConstants.ANGLER_IMPACT_ANGLE) {
      secondaryAnglerMotor.set(Constants.ElevatorConstants.ANGLER_SPEED);
    } else {
      secondaryAnglerMotor.set(0);
      anglerDone = true;
    }

    currentStep = (anglerDone) ? (currentStep = 7) : (currentStep = 6);
  }

  public void climber007() {
    /*
      9. primary retracts min
      10. secondary returns to 0 angle
    */

    boolean anglerDone = false;
    boolean elevatorDone = false;

    elevatorStepsEntry.setString("step 7");

    if (!elevatorMinSwitch.get()) {
      primaryElevatorMotor.set(Constants.ElevatorConstants.ELEVATOR_SPEED);
    } else {
      primaryElevatorMotor.set(0);
      elevatorDone = true;
    }

    if (!anglerMinSwitch.get()) {
      secondaryAnglerMotor.set(Constants.ElevatorConstants.ANGLER_SPEED);
    } else {
      secondaryAnglerMotor.set(0);
      anglerDone = true;
    }

    currentStep = (anglerDone && elevatorDone) ? (currentStep = 8) : (currentStep = 7);
  }

  public void backOneStep() {
    currentStep -= 1;
  }
}
