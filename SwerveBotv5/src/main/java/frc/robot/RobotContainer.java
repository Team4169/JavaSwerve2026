// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.Autos;
import frc.robot.commands.ExampleCommand;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.SwerveSubsystem;
import swervelib.SwerveDrive;
import swervelib.parser.SwerveParser;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.RunCommand;

import java.io.File;

import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  private final SwerveSubsystem swerveSubsystem;
  private final SwerveDrive swerveDrive;
  private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();
  private final Intake m_Intake = new Intake();
  private final Shooter m_Shooter = new Shooter();

  private final CommandXboxController m_driverController =
      new CommandXboxController(OperatorConstants.kDriverControllerPort);
  private final CommandXboxController m_operatorController =
      new CommandXboxController(OperatorConstants.kOperatorControllerPort);

  public RobotContainer() {
    System.out.println(">>> RobotContainer constructor START");
    try {
      swerveDrive = new SwerveParser(new File(Filesystem.getDeployDirectory(), "swerve"))
          .createSwerveDrive(Units.feetToMeters(15));
      System.out.println(">>> SwerveDrive created successfully");
    } catch (Exception e) {
      System.out.println(">>> SwerveDrive FAILED: " + e.getMessage());
      throw new RuntimeException("Failed to initialize swerve drive", e);
    }

    swerveSubsystem = new SwerveSubsystem(swerveDrive);
    System.out.println(">>> SwerveSubsystem created successfully");
    configureBindings();
  }

  private void configureBindings() {
    swerveSubsystem.setDefaultCommand(
        new RunCommand(
            () -> {
              try {
                SmartDashboard.putBoolean("Drive Command Running", true);

                double rawLeftY = m_driverController.getLeftY();
                double rawLeftX = m_driverController.getLeftX();
                double rawRightX = m_driverController.getRightX();

                double maxSpeed = swerveDrive.getMaximumChassisVelocity();
                double maxAngularSpeed = swerveDrive.getMaximumChassisAngularVelocity();
                SmartDashboard.putNumber("Max Speed", maxSpeed);
                SmartDashboard.putNumber("Max Angular Speed", maxAngularSpeed);

                double xSpeed = MathUtil.applyDeadband(rawLeftY, 0.1) * maxSpeed;
                double ySpeed = MathUtil.applyDeadband(rawLeftX, 0.1) * maxSpeed;
                double rot = MathUtil.applyDeadband(rawRightX, 0.1) * maxAngularSpeed;
                SmartDashboard.putNumber("xSpeed", xSpeed);
                SmartDashboard.putNumber("ySpeed", ySpeed);
                SmartDashboard.putNumber("rot", rot);

                swerveSubsystem.drive(new Translation2d(xSpeed, ySpeed), rot, false, false);
              } catch (Exception e) {
                SmartDashboard.putString("Drive Error", e.getMessage());
              }
            },
            swerveSubsystem));

    new Trigger(m_exampleSubsystem::exampleCondition)
        .onTrue(new ExampleCommand(m_exampleSubsystem));

    m_operatorController.y().whileTrue(
        Commands.parallel(m_Shooter.runShooterAndFeedHeld(), m_Intake.runIntakeHeld()));
    m_operatorController.a().whileTrue(m_Intake.runIntakeHeld());
    m_operatorController.leftBumper().whileTrue(m_Intake.foldUpHeld());
    m_operatorController.rightBumper().whileTrue(m_Intake.foldDownHeld());
    m_operatorController.start().onTrue(Commands.runOnce(swerveSubsystem::zeroHeading, swerveSubsystem));

    // Pit checklist (disabled): X -> align modules forward, B -> initialize + snapshot + delta suggestions.
    // Back captures snapshot + delta suggestions again without reinitializing.
    m_operatorController.b().onTrue(Commands.sequence(
      Commands.runOnce(swerveSubsystem::initializeToDefaultPositions, swerveSubsystem),
      Commands.runOnce(swerveSubsystem::publishAbsoluteEncoderSnapshot, swerveSubsystem),
      Commands.runOnce(swerveSubsystem::publishOffsetDeltaSuggestions, swerveSubsystem)));
    m_operatorController.back().onTrue(Commands.sequence(
      Commands.runOnce(swerveSubsystem::publishAbsoluteEncoderSnapshot, swerveSubsystem),
      Commands.runOnce(swerveSubsystem::publishOffsetDeltaSuggestions, swerveSubsystem)));
    m_operatorController.x().onTrue(Commands.runOnce(swerveSubsystem::pointModulesForward, swerveSubsystem));
  }

  public Command getAutonomousCommand() {
    return Autos.exampleAuto(m_exampleSubsystem);
  }
}