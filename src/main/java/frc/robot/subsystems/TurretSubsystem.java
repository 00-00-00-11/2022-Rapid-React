package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.utility.SparkMaxUtility;

public class TurretSubsystem extends SubsystemBase {

    private CANSparkMax turretMotor;
    private Encoder turretEncoder;
    public TurretSubsystem() {
        turretMotor = SparkMaxUtility.constructSparkMax(Constants.RobotMap.TURRET_TURN_CAN, true);
        turretEncoder = turretMotor.getEncoder();
    }

    public void runTurret(double speed) {
        ticks = turretEncoder.get();
        angle = ticks*(360/1024);
        if (angle < 90) and (if angle > -90) 
        {
            SparkMaxUtility.runSparkMax(turretMotor, speed);    
        }
        else 
        {
            SparkMaxUtility.runSparkMax(turretMotor, 0);
        }
    }
}
