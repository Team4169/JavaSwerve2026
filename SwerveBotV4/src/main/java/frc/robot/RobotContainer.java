// // Copyright (c) FIRST and other WPILib contributors.
// // Open Source Software; you can modify and/or share it under the terms of
// // the WPILib BSD license file in the root directory of this project.

// package frc.robot;

// import java.io.File;

// import com.pathplanner.lib.auto.AutoBuilder;
// import com.pathplanner.lib.auto.NamedCommands;

// import edu.wpi.first.math.MathUtil;
// import edu.wpi.first.math.geometry.Translation2d;
// import edu.wpi.first.math.util.Units;
// import edu.wpi.first.wpilibj.Filesystem;
// import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
// import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
// import edu.wpi.first.wpilibj2.command.Command;
// import edu.wpi.first.wpilibj2.command.CommandScheduler;
// import edu.wpi.first.wpilibj2.command.Commands;
// import edu.wpi.first.wpilibj2.command.RunCommand;
// import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
// import edu.wpi.first.wpilibj2.command.button.Trigger;
// import frc.robot.Constants.OperatorConstants;
// import frc.robot.commands.ExampleCommand;
// import frc.robot.subsystems.ExampleSubsystem;
// import frc.robot.subsystems.Intake;
// import frc.robot.subsystems.Shooter;
// import frc.robot.subsystems.SwerveSubsystem;
// import swervelib.SwerveDrive;
// import swervelib.parser.SwerveParser;

// /**
//  * This class is where the bulk of the robot should be declared. Since Command-based is a
//  * "declarative" paradigm, very little robot logic should actually be handled in the {@link Robot}
//  * periodic methods (other than the scheduler calls). Instead, the structure of the robot (including
//  * subsystems, commands, and trigger mappings) should be declared here.
//  */
// public class RobotContainer {
//   private final SwerveSubsystem swerveSubsystem;
//   private SwerveDrive swerveDrive;
//     private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();
//     private final SendableChooser<Command> autoChooser;
//     private final Intake m_Intake = new Intake();
//     private final Shooter m_Shooter = new Shooter();
  
//     private final CommandXboxController m_driverController =
//         new CommandXboxController(OperatorConstants.kDriverControllerPort);
//     private final CommandXboxController m_operatorController =
//         new CommandXboxController(OperatorConstants.kOperatorControllerPort);
  
//     public RobotContainer() {
//       System.out.println(">>> RobotContainer constructor START");
//       try {
//         swerveDrive = new SwerveParser(new File(Filesystem.getDeployDirectory(), "swerve"))
//             .createSwerveDrive(Units.feetToMeters(15));
//         System.out.println(">>> SwerveDrive created successfully");
//       } catch (Exception e) {
//         System.out.println(">>> SwerveDrive FAILED: " + e.getMessage());
//         throw new RuntimeException("Failed to initialize swerve drive", e);
//       }
  
//       swerveSubsystem = new SwerveSubsystem(swerveDrive);
//       System.out.println(">>> SwerveSubsystem created successfully");
//       System.out.println(">>> RobotContainer constructor START");
//       NamedCommands.registerCommand("shoot", Commands.parallel(m_Shooter.runShooterAndFeedHeld(), m_Intake.runIntakeHeld()));
//       NamedCommands.registerCommand("intake", m_Intake.runIntakeHeld());
//       try {
//         swerveDrive = new SwerveParser(new File(Filesystem.getDeployDirectory(), "swerve"))
//           .createSwerveDrive(Units.feetToMeters(15));
//     } catch (Exception e) {
//       throw new RuntimeException("Failed to initialize swerve drive", e);
//     }
//     //swerveSubsystem = new SwerveSubsystem(swerveDrive);
//     autoChooser = AutoBuilder.buildAutoChooser();
//     SmartDashboard.putData("Auto Chooser", autoChooser);
//     configureBindings();
//   }

//   private void configureBindings() {
//     swerveSubsystem.setDefaultCommand(
//         new RunCommand(
//             () -> {
//               try {
//                 SmartDashboard.putBoolean("Drive Command Running", true);

//                 double rawLeftY = m_driverController.getLeftY();
//                 double rawLeftX = m_driverController.getLeftX();
//                 double rawRightX = m_driverController.getRightX();

//                 double maxSpeed = swerveDrive.getMaximumChassisVelocity();
//                 double maxAngularSpeed = swerveDrive.getMaximumChassisAngularVelocity();
//                 SmartDashboard.putNumber("Max Speed", maxSpeed);
//                 SmartDashboard.putNumber("Max Angular Speed", maxAngularSpeed);

//                 double xSpeed = MathUtil.applyDeadband(rawLeftY, 0.1) * maxSpeed;
//                 double ySpeed = MathUtil.applyDeadband(rawLeftX, 0.1) * maxSpeed;
//                 double rot = MathUtil.applyDeadband(rawRightX, 0.1) * maxAngularSpeed;
//                 SmartDashboard.putNumber("xSpeed", xSpeed);
//                 SmartDashboard.putNumber("ySpeed", ySpeed);
//                 SmartDashboard.putNumber("rot", rot);

//                 swerveSubsystem.drive(new Translation2d(xSpeed, ySpeed), rot, false, false);
//               } catch (Exception e) {
//                 SmartDashboard.putString("Drive Error", e.getMessage());
//               }
//             },
//             swerveSubsystem));

//     new Trigger(m_exampleSubsystem::exampleCondition)
//         .onTrue(new ExampleCommand(m_exampleSubsystem));

//     m_operatorController.y().whileTrue(
//         Commands.parallel(m_Shooter.runShooterAndFeedHeld(), m_Intake.runIntakeHeld()));
//     m_operatorController.a().whileTrue(m_Intake.runIntakeHeld());
//     m_operatorController.leftBumper().whileTrue(m_Intake.foldUpHeld());
//     m_operatorController.rightBumper().whileTrue(m_Intake.foldDownHeld());
//     m_operatorController.start().onTrue(Commands.runOnce(swerveSubsystem::zeroHeading, swerveSubsystem));
//     m_operatorController.b().onTrue(
//         Commands.runOnce(swerveSubsystem::initializeToDefaultPositions, swerveSubsystem));
//     m_operatorController.x().onTrue(Commands.runOnce(swerveSubsystem::pointModulesForward, swerveSubsystem));
//   }

//   public Command getAutonomousCommand() {
//     return autoChooser.getSelected();
//     //CommandScheduler.getInstance().schedule(m_autonomousCommand);
//   }
// }

// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import java.io.File;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Filesystem;
import edu.wpi.first.wpilibj.smartdashboard.SendableChooser;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.RunCommand;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc.robot.Constants.OperatorConstants;
import frc.robot.commands.ExampleCommand;
import frc.robot.subsystems.ExampleSubsystem;
import frc.robot.subsystems.Intake;
import frc.robot.subsystems.Shooter;
import frc.robot.subsystems.SwerveSubsystem;
import swervelib.SwerveDrive;
import swervelib.parser.SwerveParser;

/**
 * This class is where the bulk of the robot should be declared. Since Command-based is a
 * "declarative" paradigm, very little robot logic should actually be handled in the Robot
 * periodic methods (other than the scheduler calls). Instead, the structure of the robot
 * (including subsystems, commands, and trigger mappings) should be declared here.
 */
public class RobotContainer {
  private final SwerveSubsystem swerveSubsystem;
  private final SwerveDrive swerveDrive;

  private final ExampleSubsystem m_exampleSubsystem = new ExampleSubsystem();
  private final SendableChooser<Command> autoChooser;
  private final Intake m_Intake = new Intake();
  private final Shooter m_Shooter = new Shooter();

  private final CommandXboxController m_driverController =
      new CommandXboxController(OperatorConstants.kDriverControllerPort);
  private final CommandXboxController m_operatorController =
      new CommandXboxController(OperatorConstants.kOperatorControllerPort);

  public RobotContainer() {
    System.out.println(">>> RobotContainer constructor START");

    // Initialize swerve drive ONCE
    SwerveDrive tempDrive;
    try {
      tempDrive = new SwerveParser(new File(Filesystem.getDeployDirectory(), "swerve"))
          .createSwerveDrive(Units.feetToMeters(15));
      System.out.println(">>> SwerveDrive created successfully");
    } catch (Exception e) {
      System.out.println(">>> SwerveDrive FAILED: " + e.getMessage());
      throw new RuntimeException("Failed to initialize swerve drive", e);
    }
    swerveDrive = tempDrive;

    // Create subsystem from the single swerveDrive instance
    swerveSubsystem = new SwerveSubsystem(swerveDrive);
    System.out.println(">>> SwerveSubsystem created successfully");

    // Register named commands BEFORE building auto chooser
    NamedCommands.registerCommand(
        "shoot",
        Commands.parallel(m_Shooter.runShooterAndFeedHeld(), m_Intake.runIntakeHeld()));
    NamedCommands.registerCommand("intake", m_Intake.runIntakeHeld());

    // Build auto chooser AFTER named commands are registered
    autoChooser = AutoBuilder.buildAutoChooser();
    SmartDashboard.putData("Auto Chooser", autoChooser);

    configureBindings();

    System.out.println(">>> RobotContainer constructor DONE");
  }

  private void configureBindings() {
    // Default drive command
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

    // Example subsystem trigger
    new Trigger(m_exampleSubsystem::exampleCondition)
        .onTrue(new ExampleCommand(m_exampleSubsystem));

    // Operator controls
    m_operatorController.y().whileTrue(
        Commands.parallel(m_Shooter.runShooterAndFeedHeld(), m_Intake.runIntakeHeld()));
    m_operatorController.a().whileTrue(m_Intake.runIntakeHeld());
    m_operatorController.leftBumper().whileTrue(m_Intake.foldUpHeld());
    m_operatorController.rightBumper().whileTrue(m_Intake.foldDownHeld());

    // Utility commands
    m_operatorController.start().onTrue(
        Commands.runOnce(swerveSubsystem::zeroHeading, swerveSubsystem));
    m_operatorController.b().onTrue(
        Commands.runOnce(swerveSubsystem::initializeToDefaultPositions, swerveSubsystem));
    m_operatorController.x().onTrue(
        Commands.runOnce(swerveSubsystem::pointModulesForward, swerveSubsystem));
  }

  public Command getAutonomousCommand() {
    return autoChooser.getSelected();
  }
}