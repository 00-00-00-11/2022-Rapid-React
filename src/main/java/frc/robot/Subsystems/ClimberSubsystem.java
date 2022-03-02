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
import edu.wpi.first.wpilibj.CAN;
import edu.wpi.first.wpilibj.DigitalInput;
import edu.wpi.first.wpilibj.Joystick;
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

  int currentStep = 0;

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
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  // button hit by driver, elevator height to max
  public void elevatorUpDriver(PS4Controller joystick) {
    anglerMotorLeft.set(joystick.getRightY());
    anglerMotorRight.set(joystick.getRightY());

    elevatorMotor.set(joystick.getL2Axis() - joystick.getR2Axis());
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
    lastFinishedStep += 1;
  }

  public void prevStep() { // ElevatorPrevStep
    currentStep -= 1;
    lastFinishedStep -= 1;
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

    if (currentStep < 1) {     //runs one after currentStep
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
  
  //return methods
  public int getCurrentStep() {
    return currentStep;
  }

  public int getLastFinishedStep() {
    return lastFinishedStep;
  }

  //overloaded method: for finish process insert
  /*
  public void finishPrimaryProcess(CANSparkMax primaryMotor, boolean anglerVariable){
    primaryElevatorMotor00.set(0);
    anglerVariable=true;
  }
  public void finishProcess(CANSparkMax primaryMotor, CANSparkMax secondaryMotor, boolean anglerVariable) {
    primaryElevat
  }

  */

  //possibly do single method with conditional processing
  /*
  public void finishProcess(CANSparkMax primaryMotor,....)
    if(primarymotor = true)

  */


public void iterateLastFinishedStep(boolean anglerDone_elevatorDone) {
  if(anglerDone_elevatorDone){ 
    lastFinishedStep+=1;
  }
}

  public void iterateLastFinishedStep(boolean anglerDone_elevatorDone, boolean elevatorDone_anglerDone) {
    if(anglerDone_elevatorDone && elevatorDone_anglerDone) {
      lastFinishedStep+=1;
    }
}

  public void resetElevatorAnglerBoolean(String elevatorOrAngler) { //maybe use another setProcess that recives one or two variables indicating booleans to change, and a boolean to indicate desired result.
    if (elevatorOrAngler.equals("elevator")) {
      elevatorDone = false; 
    } else if(elevatorOrAngler.equals("angler")) {
      anglerDone=false;
    } 
    /*    String possibility if not overloaded
    else if(elevatorOrAngler.equals("both")) {
      anglerDone=false;
      elevatorDone=false;
    }
    */
  }                                                              //do not ignore => }

/*   overloaded for both booleans??
  public void resetElevatorAnglerBoolean(String elevatorOrAngler) { 

  }

  */
  public void finishPrimaryProcess() { //for use later
    primaryElevatorMotor00.set(0.0);
    elevatorDone=true;
  }

  public void finishSecondaryProcess() { //for use later
    secondaryAnglerMotor00.set(0.0);
    anglerDone = true;
  }

  public void climber001() {
    /*
      1. primary extends small distance
      2. secondary extends small angle
    */

    elevatorDone=false;
    anglerDone=false;

    elevatorStepsEntry.setString("step 1");

    if (primaryElevatorMotor00.getEncoder().getPosition()
        < Constants.ElevatorConstants.ELEVATOR_SMALL_DISTANCE) {
      primaryElevatorMotor00.set(Constants.ElevatorConstants.ELEVATOR_SPEED);
      System.out.println(primaryElevatorMotor00.getEncoder().getPosition());
    } else {
      finishPrimaryProcess();

    }

    System.out.println("step 1 primary " + elevatorDone);

    if (secondaryAnglerMotor00.getEncoder().getPosition()
        < Constants.ElevatorConstants.ANGLER_SMALL_ANGLE) {
      secondaryAnglerMotor00.set(Constants.ElevatorConstants.ANGLER_SPEED);
      System.out.println(secondaryAnglerMotor00.getEncoder().getPosition());
    } else {
      finishSecondaryProcess();
    }

    System.out.println("step 1 secondary " + anglerDone);

    iterateLastFinishedStep(elevatorDone,anglerDone);
  }

  public void climber002() {
    /*
      3. primary retracts
      4. secondary returns to 0 angle
    */

    elevatorDone=false;
    anglerDone=false;

    elevatorStepsEntry.setString("step 2");  //step made consiscely 

    if (primaryElevatorMotor00.getEncoder().getPosition() > 0) {
      primaryElevatorMotor00.set(-Constants.ElevatorConstants.ELEVATOR_SPEED);
    } else {
      finishPrimaryProcess();
    }

    if (secondaryAnglerMotor00.getEncoder().getPosition() > 0) {
      secondaryAnglerMotor00.set(-Constants.ElevatorConstants.ANGLER_SPEED); //omega 
    } else {
      finishSecondaryProcess();
    }

    System.out.println("step 2 primary" + elevatorDone);     
    System.out.println("step 2 secondary" + anglerDone);

    iterateLastFinishedStep(elevatorDone,anglerDone);
  }

  public void climber003() {
    /*
      5. primary extends partially
    */

     elevatorDone = false;

    elevatorStepsEntry.setString("step 3");

    if (primaryElevatorMotor00.getEncoder().getPosition()
        < Constants.ElevatorConstants.ELEVATOR_SMALL_DISTANCE) {
      primaryElevatorMotor00.set(Constants.ElevatorConstants.ELEVATOR_SPEED);
    } else {
      finishPrimaryProcess();
    }

    System.out.println("step 3 primary" + elevatorDone);

    iterateLastFinishedStep(elevatorDone);
  }

  public void climber004() {
    /*
      6. secondary rotates large angle
    */

     anglerDone = false;

    elevatorStepsEntry.setString("step 4");

    if (secondaryAnglerMotor00.getEncoder().getPosition()
        < Constants.ElevatorConstants.ANGLER_LARGE_ANGLE) {
      secondaryAnglerMotor00.set(Constants.ElevatorConstants.ANGLER_SPEED);
    } else {
      finishSecondaryProcess();
    }

    System.out.println("step 4 secondary" + anglerDone);

    iterateLastFinishedStep(anglerDone);
  }

  public void climber005() {
    /*
      7. primary extends max
    */

     elevatorDone = false;

    elevatorStepsEntry.setString("step 5");

    if (primaryElevatorMotor00.getEncoder().getPosition()
        < Constants.ElevatorConstants.ELEVATOR_LARGE_DISTANCE) {
      primaryElevatorMotor00.set(Constants.ElevatorConstants.ELEVATOR_SPEED);
    } else {
      finishPrimaryProcess();
    }

    System.out.println("step 5 primary" + elevatorDone);

    iterateLastFinishedStep(elevatorDone);
  }

  public void climber006() {
    /*
      8. secondary rotates till primary impacts
    */

     anglerDone = false;

    elevatorStepsEntry.setString("step 6");

    if (secondaryAnglerMotor00.getEncoder().getPosition()
        > Constants.ElevatorConstants.ANGLER_IMPACT_ANGLE) {
      secondaryAnglerMotor00.set(-Constants.ElevatorConstants.ANGLER_SPEED);
    } else {
      finishSecondaryProcess();
    }

    System.out.println("step 6 secondary" + anglerDone);

    iterateLastFinishedStep(anglerDone);
  }

  public void climber007() {
    /*
      9. primary retracts min
      10. secondary returns to 0 angle
    */

     anglerDone = false;
     elevatorDone = false;

    elevatorStepsEntry.setString("step 7");

    if (primaryElevatorMotor00.getEncoder().getPosition() > 0) {
      primaryElevatorMotor00.set(-Constants.ElevatorConstants.ELEVATOR_SPEED);
    } else {
      finishPrimaryProcess();
    }

    if (secondaryAnglerMotor00.getEncoder().getPosition() > 0) {
      secondaryAnglerMotor00.set(-Constants.ElevatorConstants.ANGLER_SPEED);
    } else {
      finishSecondaryProcess();
    }

    System.out.println("step 7 primary" + elevatorDone);  //combine into single "return done"
    System.out.println("step 7 secondary" + anglerDone);  

    iterateLastFinishedStep(anglerDone,elevatorDone);
  }
}
