// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc.robot.Constants.OperatorConstants;

import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

public class Shooter extends SubsystemBase {
  private static final int kFlywheelMotorPort = 35;
  private static final int kKickerMotorPort = 19;
  private static final int kAuxKickerMotorPort = 9;

  private final SparkFlex m_flywheelMotor;
  private final SparkFlex m_kickerMotor;
  private final SparkMax m_auxKickerMotor;
  private final CommandXboxController m_operatorController;
  public Shooter() {
    m_operatorController = new CommandXboxController(OperatorConstants.kOperatorControllerPort);
    m_flywheelMotor = new SparkFlex(kFlywheelMotorPort, MotorType.kBrushless);
    m_kickerMotor = new SparkFlex(kKickerMotorPort, MotorType.kBrushless);
    m_auxKickerMotor = new SparkMax(kAuxKickerMotorPort, MotorType.kBrushless);
  }

  public Command runIntake() {
    return runOnce(this::startFeedMotors);
  }

  public Command stopIntake() {
    return runOnce(this::stopFeedMotors);
  }

  public Command runShooter() {
    return runOnce(this::startFlywheelMotor);
  }

  public Command runShooterAndFeedHeld() {
    return startEnd(this::startShooterAndFeed, this::stopShooterAndFeed);
  }

  public Command stopShooter() {
    return runOnce(this::stopFlywheelMotor);
  }

  public boolean exampleCondition() {
    return false;
  }

  @Override
  public void periodic() {}

  @Override
  public void simulationPeriodic() {}

  private void startFeedMotors() {
    m_kickerMotor.set(0.5);
    m_auxKickerMotor.set(-0.44);
  }

  private void stopFeedMotors() {
    m_kickerMotor.stopMotor();
    m_auxKickerMotor.stopMotor();
  }

  private void startFlywheelMotor() {
    m_flywheelMotor.set(-Math.abs(m_operatorController.getLeftY())); // -0.51 before variable shooter power
  }

  private void stopFlywheelMotor() {
    m_flywheelMotor.stopMotor();
  }

  private void startShooterAndFeed() {
    startFlywheelMotor();
    startFeedMotors();
  }

  private void stopShooterAndFeed() {
    stopFlywheelMotor();
    stopFeedMotors();
  }
}
