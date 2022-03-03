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

  double ElevAbsMin = Constants.ElevatorConstants.ELEVATOR_MIN;
  double ElevRetract = Constants.ElevatorConstants.ELEVATOR_RETRACTED;
  double ElevSmallDist = Constants.ElevatorConstants.ELEVATOR_SMALL_DISTANCE;
  double ElevLargeDist = Constants.ElevatorConstants.ELEVATOR_LARGE_DISTANCE;
  double ElevExtend = Constants.ElevatorConstants.ELEVATOR_EXTENDED;
  double ElevAbsMax = Constants.ElevatorConstants.ELEVATOR_MAX;

  double ElevSpeed = Constants.ElevatorConstants.ELEVATOR_SPEED;
  
  double AngSmall = Constants.ElevatorConstants.ANGLER_SMALL_ANGLE;
  double AngLarge = Constants.ElevatorConstants.ANGLER_LARGE_ANGLE;
  double AngImpact = Constants.ElevatorConstants.ANGLER_IMPACT_ANGLE;
  
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
  }

  @Override
  public void periodic() {}

  // Manual Control
  public void elevatorDriver(PS4Controller joystick) {
    anglerMotorLeft.set(joystick.getRightY());
    anglerMotorRight.set(joystick.getRightY());

    elevatorMotor.set(joystick.getL2Axis() - joystick.getR2Axis());
  }

  public void nextStep() { currentStep += 1; }

  public void prevStep() { currentStep -= 1; }

  public void runAutoStep() {
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
    } else { // all steps complete
      System.out.println("Climber Done");
      elevatorStepsEntry.setString("complete");
    }
  }

  public boolean checkNextRungComplete() {
    return lastFinishedStep == 7;
  }

  public void resetSteps() {
    currentStep = 0;
  }
  
  // Return methods
  public int getCurrentStep() {
    return currentStep;
  }

  public int getLastFinishedStep() {
    return lastFinishedStep;
  }

  public void climber001() {
    /*
      1. primary extends small distance
      2. secondary extends small angle
    */

    boolean elevatorDone = false;
    boolean anglerLeftDone = false;
    boolean anglerRightDone = false;

    elevatorStepsEntry.setString("step 1");

    if (elevatorMotor.getEncoder().getPosition() < ElevSmallDist) {
      elevatorMotor.set(ElevSpeed);
    } else {
      elevatorMotor.set(0);
      elevatorDone = true;
    }

    if (anglerMotorLeft.getEncoder().getPosition() < AngLarge) {
      anglerMotorLeft.set(AnglerSpeed);
    } else {
      anglerLeftDone = true;
    }

    if (anglerMotorRight.getEncoder().getPosition() < AngLarge) {
      anglerMotorRight.set(AnglerSpeed);
    } else {
      anglerRightDone = true;
    }

    if(elevatorDone && anglerLeftDone && anglerRightDone) {
      lastFinishedStep = 1;
      currentStep += 1;
    }
    
  }

  public void climber002() {
    /*
      3. primary retracts
      4. secondary returns to 0 angle
    */

    boolean elevatorDone = false;
    boolean anglerLeftDone = false;
    boolean anglerRightDone = false;

    elevatorStepsEntry.setString("step 2");
    
    if (elevatorMotor.getEncoder().getPosition() > 0) {
      elevatorMotor.set(-ElevSpeed);
    } else {
      elevatorMotor.set(0);
      elevatorDone = true;
    }
    
    if (anglerMotorLeft.getEncoder().getPosition() > 0) {
      anglerMotorLeft.set(-AnglerSpeed); 
    } else {
      anglerMotorLeft.set(0);
      anglerLeftDone = true;
    }

    if (anglerMotorRight.getEncoder().getPosition() > 0) {
      anglerMotorRight.set(-AnglerSpeed); 
    } else {
      anglerMotorRight.set(0);
      anglerRightDone = true;
    }

    if(elevatorDone && anglerLeftDone && anglerRightDone) {
      lastFinishedStep = 2;
      currentStep += 1;
    }
  }

  public void climber003() {
    /*
      5. primary extends partially
    */

    boolean elevatorDone = false;

    elevatorStepsEntry.setString("step 3");

    if (elevatorMotor.getEncoder().getPosition() < ElevSmallDist) {
      elevatorMotor.set(ElevSpeed);
    } else {
      elevatorMotor.set(0);
      elevatorDone = true;
    }
    
    if(elevatorDone) {
      lastFinishedStep = 3;
      currentStep += 1;
    }
  }

  public void climber004() {
    /*
      6. secondary rotates large angle
    */

    boolean anglerLeftDone = false;
    boolean anglerRightDone = false;

    elevatorStepsEntry.setString("step 4");

    if (anglerMotorLeft.getEncoder().getPosition() < AngLarge) {
      anglerMotorLeft.set(AnglerSpeed);
    } else {
      anglerMotorLeft.set(0);
      anglerLeftDone = true;
    }

    if (anglerMotorRight.getEncoder().getPosition() < AngLarge) {
      anglerMotorRight.set(AnglerSpeed);
    } else {
      anglerMotorRight.set(0);
      anglerRightDone = true;
    }

    if(anglerLeftDone && anglerRightDone) {
      lastFinishedStep = 4;
      currentStep += 1;
    }
  }

  public void climber005() {
    /*
      7. primary extends max
    */

    boolean elevatorDone = false;

    elevatorStepsEntry.setString("step 5");

    if (elevatorMotor.getEncoder().getPosition() < ElevLargeDist) {
      elevatorMotor.set(ElevSpeed);
    } else {
      elevatorMotor.set(0);
      elevatorDone = true;
    }

    if(elevatorDone) {
      lastFinishedStep = 5;
      currentStep += 1;
    }
  }

  public void climber006() {
    /*
      8. secondary rotates till primary impacts
    */

    boolean anglerLeftDone = false;
    boolean anglerRightDone = false;

    elevatorStepsEntry.setString("step 6");

    if (anglerMotorLeft.getEncoder().getPosition() > AngImpact) {
      anglerMotorLeft.set(-AnglerSpeed);
    } else {
      anglerMotorLeft.set(0);
      anglerLeftDone = true;
    }

    if (anglerMotorRight.getEncoder().getPosition() > AngImpact) {
      anglerMotorRight.set(-AnglerSpeed);
    } else {
      anglerMotorRight.set(0);
      anglerRightDone = true;
    }

    if(anglerLeftDone && anglerRightDone) {
      lastFinishedStep = 6;
      currentStep += 1;
    }
  }

  public void climber007() {
    /*
      9. primary retracts min
      10. secondary returns to 0 angle
    */

    boolean anglerLeftDone = false;
    boolean anglerRightDone = false;
    boolean elevatorDone = false;

    elevatorStepsEntry.setString("step 7");

    if (elevatorMotor.getEncoder().getPosition() > 0) {
      elevatorMotor.set(-ElevSpeed);
    } else {
      elevatorMotor.set(0);
      elevatorDone = true;
    }

    if (anglerMotorLeft.getEncoder().getPosition() > 0) {
      anglerMotorLeft.set(-AnglerSpeed);
    } else {
      anglerMotorLeft.set(0);
      anglerLeftDone = true;
    }

    if (anglerMotorRight.getEncoder().getPosition() > 0) {
      anglerMotorRight.set(-AnglerSpeed);
    } else {
      anglerMotorRight.set(0);
      anglerRightDone = true;
    }

    if(elevatorDone && anglerLeftDone && anglerRightDone) {
      lastFinishedStep = 7;
      currentStep += 1;
    }
  }
}

