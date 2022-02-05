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
import edu.wpi.first.networktables.*;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.shuffleboard.*;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ClimberSubsystem extends SubsystemBase {
  /** Creates a new ClimberSubsystem. */
  CANSparkMax primaryElevatorMotor;
  CANSparkMax primaryElevatorMotor2;
  CANSparkMax primaryElevatorMotor3;

  CANSparkMax secondaryAnglerMotor;
  CANSparkMax secondaryAnglerMotor2;
  CANSparkMax secondaryAnglerMotor3;

  DigitalInput elevatorMaxSwitch;
  DigitalInput elevatorMinSwitch;

  DigitalInput anglerMaxSwitch;
  DigitalInput anglerMinSwitch;

  private ShuffleboardTab tab = Shuffleboard.getTab(Constants.ElevatorConstants.SHUFFLEBOARD_TAB);
  private NetworkTableEntry elevatorStepsEntry = tab.add("Elevator Step", "").getEntry();

  boolean step001 = false;
  boolean step002 = false;
  boolean step003 = false;
  boolean step004 = false;
  boolean step005 = false;
  boolean step006 = false;
  boolean step007 = false;

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

    if (!step001) {
      climber001();
    } else if (!step002) {
      climber002();
    } else if (!step003) {
      climber003();
    } else if (!step004) {
      climber004();
    } else if (!step005) {
      climber005();
    } else if (!step006) {
      climber006();
    } else if (!step007) {
      climber007();
    } else {
      System.out.println("Climber Done");
      elevatorStepsEntry.setString("complete - ready for new loop");
    }
  }

  public boolean checkNextRungComplete() {
    return step007;
  }

  public void resetSteps() {
    step001 = false;
    step002 = false;
    step003 = false;
    step004 = false;
    step005 = false;
    step006 = false;
    step007 = false;
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

    step001 = anglerDone && elevatorDone;
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

    step002 = anglerDone && elevatorDone;
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

    step003 = elevatorDone;
  }

  public void climber004() {
    /*
      6. secondary rotates large angle
    */

    boolean secondaryDone = false;

    elevatorStepsEntry.setString("step 4");

    if (secondaryAnglerMotor.getEncoder().getPosition()
        < Constants.ElevatorConstants.ANGLER_LARGE_ANGLE) {
      secondaryAnglerMotor.set(Constants.ElevatorConstants.ANGLER_SPEED);
    } else {
      secondaryAnglerMotor.set(0);
      secondaryDone = true;
    }

    step004 = secondaryDone;
  }

  public void climber005() {
    /*
      7. primary extends max
    */

    boolean primaryDone = false;

    elevatorStepsEntry.setString("step 5");

    if (!elevatorMaxSwitch.get()) {
      primaryElevatorMotor.set(Constants.ElevatorConstants.ELEVATOR_SPEED);
    } else {
      primaryElevatorMotor.set(0);
      primaryDone = true;
    }

    step005 = primaryDone;
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

    step006 = anglerDone;
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

    step007 = elevatorDone && anglerDone;
  }
}
