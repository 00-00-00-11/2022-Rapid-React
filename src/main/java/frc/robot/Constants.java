package frc.robot;

public final class Constants {

  public final class DriveConstants {
    public static final int LEFT_MASTER_CAN = 1;
    public static final int LEFT_SLAVE_CAN1 = 2;
    public static final int LEFT_SLAVE_CAN2 = 15;
    public static final int RIGHT_MASTER_CAN = 3;
    public static final int RIGHT_SLAVE_CAN1 = 4;
    public static final int RIGHT_SLAVE_CAN2 = 18;

    public static final double DRIVE_SPEED = 0.2;
    public static final double TURN_SPEED = 0.75;

    public static final double turnKP = 0.01;
    public static final double turnKI = 0;
    public static final double turnKD = 0;
  }
}
