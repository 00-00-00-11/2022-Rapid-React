// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.CANSparkMax;
import com.revrobotics.CANSparkMaxLowLevel;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.CANPIDController;
import frc.robot.Constants;

public class ShooterSubsystem extends SubsystemBase {
    /* Shooter CANSpark Definition */ 
    CANSparkMax feederMotor = new CANSparkMax(Constants.ShooterConstants.FEEDER_MOTOR_PORT, CANSparkMaxLowLevel.MotorType.kBrushless);
    CANSparkMax flyWheelMotor = new CANSparkMax(Constants.ShooterConstants.FLY_WHEEL_MOTOR_PORT, CANSparkMaxLowLevel.MotorType.kBrushless);

    /* Shooter CANPIDController Definition */
    private CANPIDController feederPIDController = feederMotor.getPIDController();
    private CANPIDController flyWheelPIDController = flyWheelMotor.getPIDController();

    public ShooterSubsystem() {
        /* Feeder PID Controller */
        feederPIDController.setP(Constants.shooterConstants.kP);
        feederPIDController.setI(Constants.shooterConstants.kI);
        feederPIDController.setD(Constants.shooterConstants.kD);
        feederPIDController.setIZone(Constants.shooterConstants.kIz);
        feederPIDController.setFF(Constants.shooterConstants.kFF);
        feederPIDController.setOutputRange(Constants.shooterConstants.kMinOutput, Constants.shooterConstants.kMaxOutput);

        /* Fly Wheel PID Controller */
        flyWheelPIDController.setP(Constants.shooterConstants.kP);
        flyWheelPIDController.setI(Constants.shooterConstants.kI);
        flyWheelPIDController.setD(Constants.shooterConstants.kD);
        flyWheelPIDController.setIZone(Constants.shooterConstants.kIz);
        flyWheelPIDController.setFF(Constants.shooterConstants.kFF);
        flyWheelPIDController.setOutputRange(Constants.shooterConstants.kMinOutput, Constants.shooterConstants.kMaxOutput);
    }
    /* Shoots The Ball */
    public void shootBalls() {
        // Sets The Speeds From PID Constants
        double speed = Constants.shooterConstants.multiplier * Constants.shooterConstants.maxRPM;

        // Spins The Feeder And Fly Wheel Motor
        topPidController.setReference(speed, ControlType.kVelocity);
        bottomPidController.setReference(speed, ControlType.kVelocity);
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