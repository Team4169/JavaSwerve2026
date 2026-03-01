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
import edu.wpi.first.math.controller.HolonomicDriveController;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj2.command.RunCommand;
//import com.pathplannerlib.lib.util.HolonomicDriveController;


import java.io.File;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.config.PIDConstants;

import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
 * subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  // The robot's subsystems and commands are defined here...

  private final SwerveSubsystem swerveSubsystem;
  private final SwerveDrive swerveDrive;
  private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();
  private final Intake m_Intake = new Intake();
  private final Shooter m_Shooter = new Shooter();

  // Replace with CommandPS4Controller or CommandJoystick if needed
  private final CommandXboxController m_driverController =
      new CommandXboxController(OperatorConstants.kDriverControllerPort);
  private final CommandXboxController m_operatorController =
      new CommandXboxController(OperatorConstants.kOperatorControllerPort);

  /** The container for the robot. Contains subsystems, OI devices, and commands. */
  public RobotContainer() {
    System.out.println(">>> RobotContainer constructor START");
    try {
      swerveDrive = new SwerveParser(
        new File(Filesystem.getDeployDirectory(), "swerve")
      ).createSwerveDrive(Units.feetToMeters(15)); //argument is max speed
      System.out.println(">>> SwerveDrive created successfully");
    } catch (Exception e) {
      System.out.println(">>> SwerveDrive FAILED: " + e.getMessage());
      throw new RuntimeException("Failed to initialize swerve drive", e);
    }

    swerveSubsystem = new SwerveSubsystem(swerveDrive);
    System.out.println(">>> SwerveSubsystem created successfully");
    //AutoBuilder.configureHolonomic(
    //  swerveSubsystem::getPose,
    //  swerveSubsystem::resetPose,
    //  swerveSubsystem::getSpeeds,
    //  swerveSubsystem::driveRobotRelative,
    //  new HolonomicPathFollowerConfig(
    //      new PIDConstants(5.0, 0.0, 0.0),
    //      new PIDConstants(5.0, 0.0, 0.0),
    //    4.5,
    //     0.4
    //  )
    //);
    // Configure the trigger bindings
    configureBindings();
  }

  /**
   * Use this method to define your trigger->command mappings. Triggers can be created via the
   * {@link Trigger#Trigger(java.util.function.BooleanSupplier)} constructor with an arbitrary
   * predicate, or via the named factories in {@link
   * edu.wpi.first.wpilibj2.command.button.CommandGenericHID}'s subclasses for {@link
   * CommandXboxController Xbox}/{@link edu.wpi.first.wpilibj2.command.button.CommandPS4Controller
   * PS4} controllers or {@link edu.wpi.first.wpilibj2.command.button.CommandJoystick Flight
   * joysticks}.
   */
  private void configureBindings() {

    swerveSubsystem.setDefaultCommand(
      new RunCommand(
        () -> {
          try {
            SmartDashboard.putBoolean("Drive Command Running", true);

            double rawLeftY = m_driverController.getLeftY();
            double rawLeftX = m_driverController.getLeftX();
            double rawRightX = m_driverController.getRightX();
            SmartDashboard.putNumber("Raw LeftY", rawLeftY);
            SmartDashboard.putNumber("Raw LeftX", rawLeftX);
            SmartDashboard.putNumber("Raw RightX", rawRightX);

            double maxSpeed = swerveDrive.getMaximumChassisVelocity();
            double maxAngularSpeed = swerveDrive.getMaximumChassisAngularVelocity();
            SmartDashboard.putNumber("Max Speed", maxSpeed);
            SmartDashboard.putNumber("Max Angular Speed", maxAngularSpeed);

            double xSpeed = -MathUtil.applyDeadband(rawLeftY, 0.1) * maxSpeed;
            double ySpeed = -MathUtil.applyDeadband(rawLeftX, 0.1) * maxSpeed;
            double rot = -MathUtil.applyDeadband(rawRightX, 0.1) * maxAngularSpeed;
            SmartDashboard.putNumber("xSpeed", xSpeed);
            SmartDashboard.putNumber("ySpeed", ySpeed);
            SmartDashboard.putNumber("rot", rot);

            swerveSubsystem.drive(
              new Translation2d(ySpeed, xSpeed),
              rot,
              true,
              false
            );
          } catch (Exception e) {
            SmartDashboard.putString("Drive Error", e.getMessage());
          }
        },
        swerveSubsystem
      )
    );

    // Schedule `ExampleCommand` when `exampleCondition` changes to `true`
    new Trigger(m_exampleSubsystem::exampleCondition)
        .onTrue(new ExampleCommand(m_exampleSubsystem));

    // Schedule `exampleMethodCommand` when the Xbox controller's B button is pressed,
    // cancelling on release.
    m_driverController.b().whileTrue(m_Shooter.runIntake());
    m_driverController.a().whileTrue(m_Shooter.stopIntake());
    m_driverController.y().whileTrue(m_Shooter.runShooter());
    m_driverController.x().whileTrue(m_Shooter.stopShooter());
    m_operatorController.y().whileTrue((m_Intake.foldupIntake()));
    m_operatorController.a().whileTrue((m_Intake.folddownIntake()));
    m_driverController.leftBumper().whileTrue((m_Intake.runIntake()));
    
  }

  /**
   * Use this to pass the autonomous command to the main {@link Robot} class.
   *
   * @return the command to run in autonomous
   */
  public Command getAutonomousCommand() {
    // An example command will be run in autonomous
    return Autos.exampleAuto(m_exampleSubsystem);
  }
}
