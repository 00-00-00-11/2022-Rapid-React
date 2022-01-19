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

    public ShooterSubsystem() {
    }
    /* Shoots the ball */
    public void shootBalls(double feeder_speed, double fly_wheel_speed) {
        feederMotor.set(feeder_speed);
        flyWheelMotor.set(fly_wheel_speed);
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