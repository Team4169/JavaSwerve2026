// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

public class Intake extends SubsystemBase {

  private static final int kIntakeMotorPort = 7;
  private static final int kFoldMotorPort = 2;

  private final SparkMax m_intakeMotor;
  private final SparkMax m_foldMotor;

  public Intake() {
    m_intakeMotor = new SparkMax(kIntakeMotorPort, MotorType.kBrushless);
    m_foldMotor = new SparkMax(kFoldMotorPort, MotorType.kBrushless);
  }

  public Command runIntake() {
    return runOnce(this::startIntakeMotor);
  }

  public Command runIntakeHeld() {
    return startEnd(this::startIntakeMotor, this::stopIntakeMotor);
  }

  public Command foldupIntake() {
    return runOnce(this::startFoldUpMotor);
  }

  public Command foldUpHeld() {
    return startEnd(this::startFoldUpMotor, this::stopFoldMotor);
  }

  public Command folddownIntake() {
    return runOnce(this::startFoldDownMotor);
  }

  public Command foldDownHeld() {
    return startEnd(this::startFoldDownMotor, this::stopFoldMotor);
  }

  public Command ReverseIntake() {
    return startEnd(this::startIntakeMotorRev, this::stopIntakeMotor);
  }

  private void startIntakeMotor() {
    m_intakeMotor.set(-1.0);
  }

  private void startIntakeMotorRev() {
    m_intakeMotor.set(1.0);
  }

  private void stopIntakeMotor() {
    m_intakeMotor.stopMotor();
  }

  private void startFoldUpMotor() {
    m_foldMotor.set(-0.6);
  }

  private void startFoldDownMotor() {
    m_foldMotor.set(0.3);
  }

  private void stopFoldMotor() {
    m_foldMotor.stopMotor();
  }
}
