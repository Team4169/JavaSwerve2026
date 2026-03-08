// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.util.sendable.SendableRegistry;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import frc.robot.Auto;

/**
 * This is a demo program showing the use of the DifferentialDrive class. Runs the motors with tank
 * steering and an Xbox controller.
 */
public class Robot extends TimedRobot {
  private static final double kShooterSpinupDelaySeconds = 0.5;

  private final SparkMax m_frontLeftMotor =
      new SparkMax(Constants.MechanismConstants.frontleftMotorCanId, MotorType.kBrushless);
  private final SparkMax m_frontRightMotor =
      new SparkMax(Constants.MechanismConstants.frontrightMotorCanId, MotorType.kBrushless);
  private final SparkMax m_backLeftMotor =
      new SparkMax(Constants.MechanismConstants.backleftMotorCanId, MotorType.kBrushless);
  private final SparkMax m_backRightMotor =
      new SparkMax(Constants.MechanismConstants.backrightMotorCanId, MotorType.kBrushless);
  
  private final SparkMax m_backRightTurnMotor =
      new SparkMax(Constants.MechanismConstants.backrightTurnMotorCanId, MotorType.kBrushless);
  private final SparkMax m_backLeftTurnMotor =
      new SparkMax(Constants.MechanismConstants.backleftTurnMotorCanId, MotorType.kBrushless);
  private final SparkMax m_frontRightTurnMotor =
      new SparkMax(Constants.MechanismConstants.frontrightTurnMotorCanId, MotorType.kBrushless);
  private final SparkMax m_frontLeftTurnMotor =
      new SparkMax(Constants.MechanismConstants.frontleftTurnMotorCanId, MotorType.kBrushless);
      
  private final DifferentialDrive m_frontRobotDrive =
      new DifferentialDrive(m_frontLeftMotor::set, m_frontRightMotor::set);
  private final DifferentialDrive m_backRobotDrive =
      new DifferentialDrive(m_backLeftMotor::set, m_backRightMotor::set);

  private final XboxController m_driverController =
      new XboxController(Constants.OperatorConstants.driverControllerPort);
  private final XboxController m_operatorController =
      new XboxController(Constants.OperatorConstants.operatorControllerPort);

  private final SparkMax intakeMotor =
      new SparkMax(Constants.MechanismConstants.intakeMotorCanId, MotorType.kBrushless);
  private final SparkMax intakeFoldMotor =
      new SparkMax(Constants.MechanismConstants.intakeFoldMotorCanId, MotorType.kBrushless);
  private final SparkFlex shooterMotor =
      new SparkFlex(Constants.MechanismConstants.shooterFlywheelCanId, MotorType.kBrushless);
  private final SparkFlex kickerMainMotor =
      new SparkFlex(Constants.MechanismConstants.kickerMainCanId, MotorType.kBrushless);
  private final SparkMax kickerAuxMotor =
      new SparkMax(Constants.MechanismConstants.kickerAuxCanId, MotorType.kBrushless);

  private double shootSequenceStartTime = -1.0;

  private Auto m_auto;

  /** Called once at the beginning of the robot program. */
  public Robot() {
    SendableRegistry.addChild(m_frontRobotDrive, m_frontLeftMotor);
    SendableRegistry.addChild(m_frontRobotDrive, m_frontRightMotor);
    SendableRegistry.addChild(m_backRobotDrive, m_backLeftMotor);
    SendableRegistry.addChild(m_backRobotDrive, m_backRightMotor);

    // We need to invert one side of the drivetrain so that positive voltages
    // result in both sides moving forward. Depending on how your robot's
    // gearbox is constructed, you might have to invert the left side instead.
    m_frontRightMotor.setInverted(true);
    m_backRightMotor.setInverted(true);

    m_auto = new Auto(
    m_frontLeftMotor,
    m_frontRightMotor,
    m_backLeftMotor,
    m_backRightMotor,
    m_frontRobotDrive,
    m_backRobotDrive,
    intakeMotor,
    intakeFoldMotor,
    shooterMotor,
    kickerMainMotor,
    kickerAuxMotor
);
  }

  @Override
  public void teleopPeriodic() {
    m_backLeftMotor.set(0);
    m_backRightMotor.set(0);
    m_frontLeftMotor.set(0);
    m_frontRightMotor.set(0);

    double leftSpeed = -m_driverController.getLeftY();
    double rightSpeed = -m_driverController.getRightY();

    m_frontRobotDrive.tankDrive(leftSpeed, rightSpeed);
    m_backRobotDrive.tankDrive(leftSpeed, rightSpeed);

    boolean runFullShootSequence = m_operatorController.getRawButton(Constants.OperatorConstants.shootButton);
    boolean runIntakeOnly = m_operatorController.getRawButton(Constants.OperatorConstants.intakeButton);
    boolean foldUp = m_operatorController.getRawButton(Constants.OperatorConstants.foldUpButton);
    boolean foldDown = m_operatorController.getRawButton(Constants.OperatorConstants.foldDownButton);

    if (runFullShootSequence) {
      if (shootSequenceStartTime < 0.0) {
        shootSequenceStartTime = Timer.getFPGATimestamp();
      }

      shooterMotor.set(-0.51);

      boolean shooterSpunUp =
          Timer.getFPGATimestamp() - shootSequenceStartTime >= kShooterSpinupDelaySeconds;
      if (shooterSpunUp) {
        kickerMainMotor.set(0.5);
        kickerAuxMotor.set(-0.44);
        intakeMotor.set(-0.8);
      } else {
        kickerMainMotor.stopMotor();
        kickerAuxMotor.stopMotor();
        intakeMotor.stopMotor();
      }
    } else {
      shootSequenceStartTime = -1.0;
      shooterMotor.stopMotor();
      kickerMainMotor.stopMotor();
      kickerAuxMotor.stopMotor();

      if (runIntakeOnly) {
        intakeMotor.set(-0.8);
      } else {
        intakeMotor.stopMotor();
      }
    }

    if (foldUp == foldDown) {
      intakeFoldMotor.stopMotor();
    } else if (foldUp) {
      intakeFoldMotor.set(-0.25);
    } else {
      intakeFoldMotor.set(0.1);
    }
  }

  @Override
  public void autonomousInit() {
      m_auto.init();
  }

  @Override
  public void autonomousPeriodic() {
      m_auto.update();
  }

  @Override
  public void autonomousExit() {
      m_auto.stop();
}
  @Override
  public void disabledInit() {
    stopMechanisms();
  }

  @Override
  public void disabledPeriodic() {
    stopMechanisms();
  }

  private void stopMechanisms() {
    intakeMotor.stopMotor();
    intakeFoldMotor.stopMotor();
    shooterMotor.stopMotor();
    kickerMainMotor.stopMotor();
    kickerAuxMotor.stopMotor();
  }
}
