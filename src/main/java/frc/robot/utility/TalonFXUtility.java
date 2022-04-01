package frc.robot.utility;

import com.ctre.phoenix.motorcontrol.can.TalonFX;

public class TalonFXUtility {

    /* Construct Talon FX */
    public static TalonFX constructTalonFX(int canID) {
        return new TalonFX(canID);
    }
}
