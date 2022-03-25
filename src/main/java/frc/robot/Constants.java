package frc.robot;

public final class Constants {

  public final class FieldConstants {
    public static final double HIGH_GOAL_HEIGHT = 108;
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

    /* TURRET SUBSYSTEM */
    public static final int TURRET_TURN_CAN = 25;

    /* CLIMBER SUBSYSTEM */
    public static final int CLIMBER_LINEAR_CAN = 14;
    public static final int CLIMBER_LEFT_ARM_CAN = 99;
    public static final int CLIMBER_RIGHT_ARM_CAN = 12;

    /* PNEUMATICS SUBSYSTEM */
    public static final int HUB_CAN = 15;
    public static final int HUB_SOLENOID1_1 = 14;
    public static final int HUB_SOLENOID1_2 = 15;
    public static final int HUB_SOLENOID2_1 = 13;
    public static final int HUB_SOLENOID2_2 = 12;
  }
  
  public final class DriveConstants {

    public static final double DRIVE_SPEED = .5d;
    public static final double TURN_SPEED = .5d;

    public static final double turnKP = -0.0225;
    public static final double turnKI = 0;
    public static final double turnKD = 0;
    public static final double QUICK_TURN_TOLERANCE = 1;

    public static final double WHEEL_DIAMETER = 6.0;
    public static final double WHEEL_CIRCUMFERENCE = Math.PI * WHEEL_DIAMETER;
    public static final double GEAR_RATIO = 10.71;

    public static final double ksVolts = .145;
    public static final double kvVoltSecondsPerMeter = 2.8;
    public static final double kaVoltSecondsSquaredPerMeter = .425;

    public static final double jKg_METERS_SQUARED = 7.5; 
    public static final double ROBOT_MASS = 60; 
    public static final double TRACK_WIDTH = 0.7112; 

    public static final double kP = 0.01;
    public static final double kI = 0.0;
    public static final double kD = 0.0;
    public static final double DEADZONE = 0.25;
  }

  public final class ShooterConstants {
    public static final double kCIMSpeed = 1;
    public static final double FLYWHEEL_KP = 0;
  }
  public static final class IntakeConstants {
    public static double intakeSpeed = .5;
  }

  public final class IndexerConstants {
    public static final double indexerSpeed = 1;
  }

  public final class AutoConstants {
    public static final double AUTO_DIST = 6;
  }

  public final class VisionConstants {
    public static final double LIMELIGHT_ANGLE = 0; // get real angle
    public static final double LIMELIGHT_HEIGHT = 0; // get real height
    public static final double GOAL_HEIGHT = 104;
    public static final int PIPELINE = 0; 
    public static final double MAX_ALIGN_SPEED = 0.6;
    public static final double ALIGN_KP_X = 0.05d;
    public static final double ALIGN_KP_Y = 0.1d;
    public static final double ALIGN_THRESHOLD = 1d;
    public static final double ALIGN_SETPOINT = 0;
  }
    
  public final class ElevatorConstants {
    public static final String SHUFFLEBOARD_TAB = "drive";

    public static final double ELEVATOR_MIN = 0; 
    public static final double ELEVATOR_RETRACTED = 0.1;
    public static final double ELEVATOR_SMALL_DISTANCE = 0.2; 
    public static final double ELEVATOR_LARGE_DISTANCE = 0.5;
    public static final double ELEVATOR_EXTENDED = 0.8;
    public static final double ELEVATOR_MAX = 1; 

    public static final double ANGLER_SMALL_ANGLE = 0.1; 
    public static final double ANGLER_LARGE_ANGLE = 0.3; 
    public static final double ANGLER_IMPACT_ANGLE = 0.2; 

    public static final double ELEVATOR_SPEED = 0.2; 
    public static final double ANGLER_SPEED = 0.05; 
  }
}
