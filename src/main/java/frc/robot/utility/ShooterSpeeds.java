package frc.robot.utility;

public class ShooterSpeeds {

    double flywheelRPM;
    double feederRPM;

    public ShooterSpeeds(double flywheelRPM, double feederRPM) {
        this.flywheelRPM = flywheelRPM;
        this.feederRPM = feederRPM;
    }

    public double getFlywheelRPM() {
        return flywheelRPM;
    }

    public double getFeederRPM() {
        return feederRPM;
    }

    public void setFlywheelRPM(double flywheelRPM) {
        this.flywheelRPM = flywheelRPM;
    }

    public void setFeederRPM(double feederRPM) {
        this.feederRPM = feederRPM;
    }
}
