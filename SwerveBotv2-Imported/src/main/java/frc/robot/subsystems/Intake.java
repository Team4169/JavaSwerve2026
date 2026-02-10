// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.subsystems;

import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import com.revrobotics.spark.SparkMax;
import com.revrobotics.spark.SparkLowLevel.MotorType;

public class Intake extends SubsystemBase {

  private static final int kIntakeMotorPort = 0;
  private static final int kFoldMotorPort = 1;

  private final SparkMax m_intakeMotor;
  private final SparkMax m_foldMotor;

  public Intake() {
    m_intakeMotor = new SparkMax(kIntakeMotorPort, MotorType.kBrushless);
    m_foldMotor = new SparkMax(kIntakeMotorPort, MotorType.kBrushless);
  }

  /**
   * Example command factory method.
   *
   * @return a command
   */
  public Command runIntake() {
    // Inline construction of command goes here.
    // Subsystem::RunOnce implicitly requires `this` subsystem.
    return runOnce(
        () -> {
          m_intakeMotor.set(50);
        });
  }
  public Command foldIntake() {
    // Inline construction of command goes here.
    // Subsystem::RunOnce implicitly requires `this` subsystem.
    return runOnce(
        () -> {
          // one-time action goes here
        });
  }
}