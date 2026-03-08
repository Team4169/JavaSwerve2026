package frc.robot;

import edu.wpi.first.wpilibj.XboxController;

public final class Constants {
  private Constants() {}

  public static final class OperatorConstants {
    public static final int driverControllerPort = 0;
    public static final int operatorControllerPort = 1;
    public static final int shootButton = XboxController.Button.kY.value;
    public static final int intakeButton = XboxController.Button.kA.value;
    public static final int foldUpButton = XboxController.Button.kLeftBumper.value;
    public static final int foldDownButton = XboxController.Button.kRightBumper.value;

    private OperatorConstants() {}
  }

  public static final class DriveConstants {
    public static final int leftMotorPwmPort = 0;
    public static final int rightMotorPwmPort = 1;

    private DriveConstants() {}
  }

  public static final class MechanismConstants {
    public static final int intakeMotorCanId = 7;
    public static final int intakeFoldMotorCanId = 2;
    public static final int shooterFlywheelCanId = 35;
    public static final int kickerMainCanId = 19;
    public static final int kickerAuxCanId = 9;
    public static final int backleftMotorCanId = 17;//todo;
    public static final int backrightMotorCanId = 10;//todo;
    public static final int frontrightMotorCanId = 18;//todo;
    public static final int frontleftMotorCanId = 33;//todo;
    private MechanismConstants() {}
  }
}
