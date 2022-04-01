package frc.robot.subsystems;

import com.revrobotics.CANSparkMax;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc.robot.Constants;
import frc.robot.utility.SparkMaxUtility;

public class TurretSubsystem extends SubsystemBase {

    private CANSparkMax turretMotor;

    public TurretSubsystem() {
        turretMotor = SparkMaxUtility.constructSparkMax(Constants.RobotMap.TURRET_TURN_CAN, true);
    }

    public void runTurret(double speed) {
        SparkMaxUtility.runSparkMax(turretMotor, speed);
    }
}
