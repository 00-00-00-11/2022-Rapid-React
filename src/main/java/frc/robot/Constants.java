package frc.robot;

public final class Constants {

  public final class DriveConstants {}

  public final class ElevatorConstants {

    public static final String SHUFFLEBOARD_TAB = "drive";

    public static final int ELEVATOR_MOTOR = 1;
    public static final int ANGLER_MOTOR = 2;

    public static final int ELEVATOR_MAX_SWITCH = 3;
    public static final int ELEVATOR_MIN_SWITCH = 4;

    public static final int ANGLER_MAX_SWITCH = 5;
    public static final int ANGLER_MIN_SWITCH = 6;

    public static final double ELEVATOR_SMALL_DISTANCE = 0.1;
    public static final double ANGLER_SMALL_ANGLE = 0.1;
    public static final double ANGLER_LARGE_ANGLE = 0.3;
    public static final double ANGLER_IMPACT_ANGLE = 0.2;

    public static final double ELEVATOR_SPEED = 0.1;
    public static final double ANGLER_SPEED = 0.1;
  }

  public final class JoystickConstants {
    public static final int DRIVER_JOYSTICK_PORT = 1;
    public static final int OPERATOR_JOYSTICK_PORT = 2;

    // Driver Joystick Buttons

    // Operator Joystick Buttons
    public static final int DRIVER_ELEVATOR_BUTTON = 1;
    public static final int AUTO_ELEVATOR_BUTTON = 2;
  }
}
