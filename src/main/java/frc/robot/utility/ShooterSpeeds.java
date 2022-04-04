package frc.robot.utility;

public class ShooterSpeeds {

    double feederVelocity;
    double flywheelVelocity;

    public ShooterSpeeds(double flywheelVelocity, double feederVelocity) {
        this.feederVelocity = feederVelocity;
        this.flywheelVelocity = flywheelVelocity;
    }

    public double getFlywheelVelocity() {
        return flywheelVelocity;
    }

    public double getFeederVelocity() {
        return feederVelocity;
    }

    public void setFlywheelVelocity(double flywheelVelocity) {
        this.flywheelVelocity = flywheelVelocity;
    }

    public void setFeederVelocity(double feederVelocity) {
        this.feederVelocity = feederVelocity;
    }
}
