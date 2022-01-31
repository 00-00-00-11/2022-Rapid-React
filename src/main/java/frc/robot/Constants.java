package frc.robot;

public final class Constants {
  /* Defines all the  */
  public final class ShooterConstants {
    public static final int FEEDER_MOTOR_PORT = 5;
    public static final int FLY_WHEEL_MOTOR_PORT = 6;
    public static final double kP = 6e-5;
    public static final double kI = 0;
    public static final double kD = 0.00001;
    public static final double kIz = 0.0;
    public static final double kFF = 0.000092;
    public static final double kMaxOutput = 1;
    public static final double kMinOutput = -1;
    public static final double maxRPM = 11000;
    public static final double multiplier = .85;
    public static final double kCIMSpeed = .5;
  }

  public final class DriveConstants {
    public static final int LEFT_MASTER_CAN = 1;
    public static final int LEFT_SLAVE_CAN1 = 2;
    public static final int LEFT_SLAVE_CAN2 = 15;
    public static final int RIGHT_MASTER_CAN = 3;
    public static final int RIGHT_SLAVE_CAN1 = 4;
    public static final int RIGHT_SLAVE_CAN2 = 18;

    public static final double DRIVE_SPEED = 0.2;
    public static final double TURN_SPEED = 0.5;

    public static final double turnKP = 0.0075;
    public static final double turnKI = 0;
    public static final double turnKD = 0;

    public static final double WHEEL_DIAMETER = 6.0;
    public static final double WHEEL_CIRCUMFERENCE = Math.PI * WHEEL_DIAMETER;
    public static final double GEAR_RATIO = 10.71;

    public static final double ksVolts = 0.0;
    public static final double kvVoltSecondsPerMeter = 0.0;
    public static final double kaVoltSecondsSquaredPerMeter = 0.0;

    public static final double jKg_METERS_SQUARED = 7.5; // TODO get actual value
    public static final double ROBOT_MASS = 60; // in kg // TODO get actual value
    public static final double TRACK_WIDTH = 0.7112; // in kg // TODO get actual value

    public static final double kP = 0.0;
    public static final double kI = 0.0;
    public static final double kD = 0.0;
  }
}
