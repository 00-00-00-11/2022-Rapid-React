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
  CANSparkMax primaryElevatorMotor00;

  CANSparkMax primaryElevatorMotor01;

  CANSparkMax secondaryAnglerMotor00;
  CANSparkMax secondaryAnglerMotor01;

  DigitalInput elevatorMaxSwitch;
  DigitalInput elevatorMinSwitch;

  DigitalInput anglerMaxSwitch;
  DigitalInput anglerMinSwitch;

  private ShuffleboardTab tab = Shuffleboard.getTab(Constants.ElevatorConstants.SHUFFLEBOARD_TAB);
  private NetworkTableEntry elevatorStepsEntry = tab.add("Elevator Step", "").getEntry();

  int currentStep = 0;
  int lastFinishedStep = 0;

  public ClimberSubsystem() {

    primaryElevatorMotor00 = new CANSparkMax(1, CANSparkMaxLowLevel.MotorType.kBrushless);
    primaryElevatorMotor01 = new CANSparkMax(2, CANSparkMaxLowLevel.MotorType.kBrushless);

    secondaryAnglerMotor00 = new CANSparkMax(3, CANSparkMaxLowLevel.MotorType.kBrushless);
    secondaryAnglerMotor01 = new CANSparkMax(4, CANSparkMaxLowLevel.MotorType.kBrushless);

    // to reset encoders
    primaryElevatorMotor00.getEncoder().setPosition(0);
    secondaryAnglerMotor00.getEncoder().setPosition(0);
    primaryElevatorMotor01.getEncoder().setPosition(0);
    secondaryAnglerMotor01.getEncoder().setPosition(0);

    primaryElevatorMotor00.setInverted(false);
    secondaryAnglerMotor00.setInverted(false);

    primaryElevatorMotor01.follow(primaryElevatorMotor00);

    secondaryAnglerMotor01.follow(secondaryAnglerMotor00);

    elevatorMaxSwitch = new DigitalInput(Constants.ElevatorConstants.ELEVATOR_MAX_SWITCH);
    elevatorMinSwitch = new DigitalInput(Constants.ElevatorConstants.ELEVATOR_MIN_SWITCH);

    anglerMaxSwitch = new DigitalInput(Constants.ElevatorConstants.ANGLER_MAX_SWITCH);
    anglerMinSwitch = new DigitalInput(Constants.ElevatorConstants.ANGLER_MIN_SWITCH);
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  public void resetElevator() {
    if (primaryElevatorMotor00.getEncoder().getPosition() < 0) {
      primaryElevatorMotor00.getEncoder().setPosition(0);
    } else {
      primaryElevatorMotor00.set(
          Constants.ElevatorConstants.ELEVATOR_SPEED); // change to whatever direction
    }
  }

  public void resetAngler() {
    if (secondaryAnglerMotor00.getEncoder().getPosition() < 0) {
      secondaryAnglerMotor00.getEncoder().setPosition(0);
    } else {
      secondaryAnglerMotor00.set(Constants.ElevatorConstants.ANGLER_SPEED);
    }
  }

  public void resetAll() {
    resetAngler();
    resetElevator();
    resetSteps();
  }
  // new changes

  public void nextStep() { // ElevatorNextStep
    currentStep += 1;
  }

  public void prevStep() { // ElevatorPrevStep
    currentStep -= 1;
  }

  public void runSteps() {
    // Run autonomous commands here

    /*
    wagwan
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

    /*
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
    } else {  //all steps complete
      System.out.println("Climber Done");
      elevatorStepsEntry.setString("complete - ready for new loop");

    }

    */
    // int enteredStep = currentStep;

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
    } else { // all steps complete
      System.out.println("Climber Done");
      elevatorStepsEntry.setString("complete - ready for new loop");
    }
  }

  public boolean checkNextRungComplete() {
    return lastFinishedStep - currentStep == 1;
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

    if (primaryElevatorMotor00.getEncoder().getPosition()
        < Constants.ElevatorConstants.ELEVATOR_SMALL_DISTANCE) {
      primaryElevatorMotor00.set(Constants.ElevatorConstants.ELEVATOR_SPEED);
      System.out.println(primaryElevatorMotor00.getEncoder().getPosition());
    } else {
      primaryElevatorMotor00.set(0);
      elevatorDone = true;
    }

    System.out.println("step 1 primary " + elevatorDone);

    if (secondaryAnglerMotor00.getEncoder().getPosition()
        < Constants.ElevatorConstants.ANGLER_SMALL_ANGLE) {
      secondaryAnglerMotor00.set(Constants.ElevatorConstants.ANGLER_SPEED);
      System.out.println(secondaryAnglerMotor00.getEncoder().getPosition());
    } else {
      secondaryAnglerMotor00.set(0);
      anglerDone = true;
    }

    System.out.println("step 1 secondary " + anglerDone);

    lastFinishedStep =
        (anglerDone && elevatorDone) ? (lastFinishedStep = 1) : (lastFinishedStep = 0);
  }

  public void climber002() {
    /*
      3. primary retracts
      4. secondary returns to 0 angle
    */

    boolean elevatorDone = false;
    boolean anglerDone = false;

    elevatorStepsEntry.setString("step 2");

    if (primaryElevatorMotor00.getEncoder().getPosition() > 0) {
      primaryElevatorMotor00.set(-Constants.ElevatorConstants.ELEVATOR_SPEED);
    } else {
      primaryElevatorMotor00.set(0);
      elevatorDone = true;
    }

    if (secondaryAnglerMotor00.getEncoder().getPosition() > 0) {
      secondaryAnglerMotor00.set(-Constants.ElevatorConstants.ANGLER_SPEED);
    } else {
      secondaryAnglerMotor00.set(0);
      anglerDone = true;
    }

    System.out.println("step 2 primary" + elevatorDone);
    System.out.println("step 2 secondary" + anglerDone);

    lastFinishedStep =
        (anglerDone && elevatorDone) ? (lastFinishedStep = 2) : (lastFinishedStep = 1);
  }

  public void climber003() {
    /*
      5. primary extends partially
    */

    boolean elevatorDone = false;

    elevatorStepsEntry.setString("step 3");

    if (primaryElevatorMotor00.getEncoder().getPosition()
        < Constants.ElevatorConstants.ELEVATOR_SMALL_DISTANCE) {
      primaryElevatorMotor00.set(Constants.ElevatorConstants.ELEVATOR_SPEED);
    } else {
      primaryElevatorMotor00.set(0);
      elevatorDone = true;
    }

    System.out.println("step 3 primary" + elevatorDone);

    lastFinishedStep = (elevatorDone) ? (lastFinishedStep = 3) : (lastFinishedStep = 2);
  }

  public void climber004() {
    /*
      6. secondary rotates large angle
    */

    boolean anglerDone = false;

    elevatorStepsEntry.setString("step 4");

    if (secondaryAnglerMotor00.getEncoder().getPosition()
        < Constants.ElevatorConstants.ANGLER_LARGE_ANGLE) {
      secondaryAnglerMotor00.set(Constants.ElevatorConstants.ANGLER_SPEED);
    } else {
      secondaryAnglerMotor00.set(0);
      anglerDone = true;
    }

    System.out.println("step 4 secondary" + anglerDone);

    lastFinishedStep = (anglerDone) ? (lastFinishedStep = 4) : (lastFinishedStep = 3);
  }

  public void climber005() {
    /*
      7. primary extends max
    */

    boolean elevatorDone = false;

    elevatorStepsEntry.setString("step 5");

    if (primaryElevatorMotor00.getEncoder().getPosition()
        < Constants.ElevatorConstants.ELEVATOR_LARGE_DISTANCE) {
      primaryElevatorMotor00.set(Constants.ElevatorConstants.ELEVATOR_SPEED);
    } else {
      primaryElevatorMotor00.set(0);
      elevatorDone = true;
    }

    System.out.println("step 5 primary" + elevatorDone);

    lastFinishedStep = (elevatorDone) ? (lastFinishedStep = 5) : (lastFinishedStep = 4);
  }

  public void climber006() {
    /*
      8. secondary rotates till primary impacts
    */

    boolean anglerDone = false;

    elevatorStepsEntry.setString("step 6");

    if (secondaryAnglerMotor00.getEncoder().getPosition()
        > Constants.ElevatorConstants.ANGLER_IMPACT_ANGLE) {
      secondaryAnglerMotor00.set(-Constants.ElevatorConstants.ANGLER_SPEED);
    } else {
      secondaryAnglerMotor00.set(0);
      anglerDone = true;
    }

    System.out.println("step 6 secondary" + anglerDone);

    lastFinishedStep = (anglerDone) ? (lastFinishedStep = 6) : (lastFinishedStep = 5);
  }

  public void climber007() {
    /*
      9. primary retracts min
      10. secondary returns to 0 angle
    */

    boolean anglerDone = false;
    boolean elevatorDone = false;

    elevatorStepsEntry.setString("step 7");

    if (primaryElevatorMotor00.getEncoder().getPosition() > 0) {
      primaryElevatorMotor00.set(-Constants.ElevatorConstants.ELEVATOR_SPEED);
    } else {
      primaryElevatorMotor00.set(0);
      elevatorDone = true;
    }

    if (secondaryAnglerMotor00.getEncoder().getPosition() > 0) {
      secondaryAnglerMotor00.set(-Constants.ElevatorConstants.ANGLER_SPEED);
    } else {
      secondaryAnglerMotor00.set(0);
      anglerDone = true;
    }

    System.out.println("step 7 primary" + elevatorDone);
    System.out.println("step 7 secondary" + anglerDone);

    lastFinishedStep =
        (anglerDone && elevatorDone) ? (lastFinishedStep = 7) : (lastFinishedStep = 6);
  }
}
