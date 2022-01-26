// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import com.revrobotics.SparkMaxPIDController;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMax.ControlType;
import com.revrobotics.CANSparkMaxLowLevel;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;

public class ShooterSubsystem extends SubsystemBase {
  /* Shooter CANSpark Definition */
  CANSparkMax feederMotor =
      new CANSparkMax(
          Constants.ShooterConstants.FEEDER_MOTOR_PORT, CANSparkMaxLowLevel.MotorType.kBrushless);
  CANSparkMax flyWheelMotor =
      new CANSparkMax(
          Constants.ShooterConstants.FLY_WHEEL_MOTOR_PORT,
          CANSparkMaxLowLevel.MotorType.kBrushless);

  /* Shooter CANPIDController Definition */
  private SparkMaxPIDController feederPIDController = feederMotor.getPIDController();
  private SparkMaxPIDController flyWheelPIDController = flyWheelMotor.getPIDController();

  public ShooterSubsystem() {
    /* Feeder PID Controller */
    feederPIDController.setP(Constants.ShooterConstants.kP);
    feederPIDController.setI(Constants.ShooterConstants.kI);
    feederPIDController.setD(Constants.ShooterConstants.kD);
    feederPIDController.setIZone(Constants.ShooterConstants.kIz);
    feederPIDController.setFF(Constants.ShooterConstants.kFF);
    feederPIDController.setOutputRange(
        Constants.ShooterConstants.kMinOutput, Constants.ShooterConstants.kMaxOutput);

    /* Fly Wheel PID Controller */
    flyWheelPIDController.setP(Constants.ShooterConstants.kP);
    flyWheelPIDController.setI(Constants.ShooterConstants.kI);
    flyWheelPIDController.setD(Constants.ShooterConstants.kD);
    flyWheelPIDController.setIZone(Constants.ShooterConstants.kIz);
    flyWheelPIDController.setFF(Constants.ShooterConstants.kFF);
    flyWheelPIDController.setOutputRange(
        Constants.ShooterConstants.kMinOutput, Constants.ShooterConstants.kMaxOutput);
  }
  /* Shoots The Ball */
  public void shootBalls(boolean shoot) {
    if (shoot) {
      // Sets The Speeds From PID Constants
      double speed = Constants.ShooterConstants.multiplier * Constants.ShooterConstants.maxRPM;

      // Spins The Feeder And Fly Wheel Motor
      feederPIDController.setReference(speed, ControlType.kVelocity);
      flyWheelPIDController.setReference(speed, ControlType.kVelocity);
    } else {
      feederMotor.set(0);
      flyWheelMotor.set(0);
    }
  }

  @Override
  public void periodic() {
    // This method will be called once per scheduler run
  }

  @Override
  public void simulationPeriodic() {
    // This method will be called once per scheduler run during simulation
  }
}
