package frc.robot;

public final class Constants {

  public final class ShooterConstants {
    public static final int FEEDER_MOTOR_PORT = 10;
    public static final int FLY_WHEEL_MOTOR_PORT = 11;
    public static final double kP = 6e-5;
    public static final double kI = 0;
    public static final double kD = 0.00001;
    public static final double kIz = 0.0;
    public static final double kFF = 0.000092;
    public static final double kMaxOutput = 1;
    public static final double kMinOutput = -1;
    public static final double maxRPM = 11000;
    public static final double multiplier = .85;
    public static final double kCIMSpeed = 1;
  }

  public static final class IntakeConstants {
    public static double intakeSpeed = .6;
  }

  public final class RobotMap {
    /* CONTROLLERS AND INPUTS */
    public static final int DRIVER_CONTROLLER_PORT = 0;
    public static final int OPERATOR_CONTROLLER_PORT = 1;
    
    /* DRIVE SUBSYSTEM */
    public static final int RIGHT_MASTER_CAN = 1;
    public static final int RIGHT_SLAVE_CAN1 = 2;
    public static final int RIGHT_SLAVE_CAN2 = 3;
    public static final int LEFT_MASTER_CAN = 4;
    public static final int LEFT_SLAVE_CAN1 = 5;
    public static final int LEFT_SLAVE_CAN2 = 6;

    /* INTAKE SUBSYSTEM */
    public static final int INTAKE_CAN = 7;

    /* INDEXER SUBSYSTEM */
    public static final int INDEXER_TRANSITION_CAN = 8;
    public static final int INDEXER_BELT_CAN = 9;

    /* SHOOTER SUBSYSTEM */
    public static final int SHOOTER_FEEDER_CAN = 10;
    public static final int SHOOTER_FLYWHEEL_CAN = 11;

    /* CLIMBER SUBSYSTEM */
    public static final int CLIMBER_LINEAR_CAN = 12;
    public static final int CLIMBER_LEFT_ARM_CAN = 13;
    public static final int CLIMBER_RIGHT_ARM_CAN = 14;

    /* PNEUMATICS SUBSYSTEM */
    public static final int HUB_CAN = 15;
    public static final int HUB_SOLENOID1_1 = 1;
    public static final int HUB_SOLENOID1_2 = 0;
    public static final int HUB_SOLENOID2_1 = 2;
    public static final int HUB_SOLENOID2_2 = 3;
  }
  public final class DriveConstants {

    public static final double DRIVE_SPEED = 0.2;
    public static final double TURN_SPEED = 0.2;

    public static final double turnKP = -0.0225;
    public static final double turnKI = 0;
    public static final double turnKD = 0;
    public static final double QUICK_TURN_TOLERANCE = 1;

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

  public final class IndexerConstants {
    public static final double indexerSpeed = 0.5;
  }
}
