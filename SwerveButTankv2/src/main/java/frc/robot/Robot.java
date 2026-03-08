// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.util.sendable.SendableRegistry;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;

/**
 * This is a demo program showing the use of the DifferentialDrive class. Runs the motors with tank
 * steering and an Xbox controller.
 */
public class Robot extends TimedRobot {
  private final PWMSparkMax m_leftMotor =
      new PWMSparkMax(Constants.DriveConstants.leftMotorPwmPort);
  private final PWMSparkMax m_rightMotor =
      new PWMSparkMax(Constants.DriveConstants.rightMotorPwmPort);
  private final DifferentialDrive m_robotDrive =
      new DifferentialDrive(m_leftMotor::set, m_rightMotor::set);
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

  /** Called once at the beginning of the robot program. */
  public Robot() {
    SendableRegistry.addChild(m_robotDrive, m_leftMotor);
    SendableRegistry.addChild(m_robotDrive, m_rightMotor);

    // We need to invert one side of the drivetrain so that positive voltages
    // result in both sides moving forward. Depending on how your robot's
    // gearbox is constructed, you might have to invert the left side instead.
    m_rightMotor.setInverted(true);
  }

  @Override
  public void teleopPeriodic() {
    m_robotDrive.tankDrive(-m_driverController.getLeftY(), -m_driverController.getRightY());

    boolean runFullShootSequence = m_operatorController.getYButton();
    boolean runIntakeOnly = m_operatorController.getAButton();
    boolean foldUp = m_operatorController.getLeftBumperButton();
    boolean foldDown = m_operatorController.getRightBumperButton();

    if (runFullShootSequence) {
      shooterMotor.set(-0.51);
      kickerMainMotor.set(0.5);
      kickerAuxMotor.set(-0.44);
      intakeMotor.set(-0.8);
    } else {
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
      intakeFoldMotor.set(-0.1);
    } else {
      intakeFoldMotor.set(0.1);
    }
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
