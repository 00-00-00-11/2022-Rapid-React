package frc.robot;

import edu.wpi.first.wpilibj.PS4Controller;

public final class Constants {

  public final class DriveConstants {}

  public static final class IntakeConstants {
    public static int leftSolenoidPortForward = 1;
    public static int leftSolenoidPortReverse = 2;
    public static int rightSolenoidPortForward = 3;
    public static int rightSolenoidPortReverse = 4;

    public static int ps4Port = 0;
    public static int square = PS4Controller.Button.kSquare.value;
  }
}
