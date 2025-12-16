// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot;

import edu.wpi.first.wpilibj.Encoder;
import edu.wpi.first.wpilibj.Joystick;
import edu.wpi.first.wpilibj.TimedRobot;
import edu.wpi.first.wpilibj.motorcontrol.PWMSparkMax;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import com.ctre.phoenix.motorcontrol.ControlMode;
import com.ctre.phoenix.motorcontrol.can.TalonSRX;

/**
 * This sample program shows how to control a motor using a joystick. In the operator control part
 * of the program, the joystick is read and the value is written to the motor.
 *
 * <p>Joystick analog values range from -1 to 1 and motor controller inputs also range from -1 to 1
 * making it easy to work together.
 *
 * <p>In addition, the encoder value of an encoder connected to ports 0 and 1 is consistently sent
 * to the Dashboard.
 */
public class Robot extends TimedRobot {
  private static final int kMotorPort1 = 3;
  private static final int kMotorPort2 = 4;
  private static final int kMotorPort3 = 8;
  private static final int kMotorPort4 = 9;
  private static final int kMotorPort5 = 51;
  private static final int kJoystickPort = 0;
  private static final int kEncoderPortA = 0;
  private static final int kEncoderPortB = 1;

  private final TalonSRX m_motor1;
  private final TalonSRX m_motor2;
  private final TalonSRX m_motor3;
  private final TalonSRX m_motor4;
  private final TalonSRX m_motor5;
  private final Joystick m_joystick;
  private final Encoder m_encoder;

  /** Called once at the beginning of the robot program. */
  public Robot() {
    m_motor1 = new TalonSRX(kMotorPort1);
    m_motor2 = new TalonSRX(kMotorPort2);
    m_motor3 = new TalonSRX(kMotorPort3);
    m_motor4 = new TalonSRX(kMotorPort4);
    m_motor5 = new TalonSRX(kMotorPort5);
    m_joystick = new Joystick(kJoystickPort);
    m_encoder = new Encoder(kEncoderPortA, kEncoderPortB);
    // Use SetDistancePerPulse to set the multiplier for GetDistance
    // This is set up assuming a 6 inch wheel with a 360 CPR encoder.
    m_encoder.setDistancePerPulse((Math.PI * 6) / 360.0);
  }

  /*
   * The RobotPeriodic function is called every control packet no matter the
   * robot mode.
   */
  @Override
  public void robotPeriodic() {
    SmartDashboard.putNumber("Encoder", m_encoder.getDistance());
  }

  /** The teleop periodic function is called every control packet in teleop. */
  @Override
  public void teleopPeriodic() {
    m_motor1.set(ControlMode.PercentOutput, m_joystick.getY());
    m_motor2.set(ControlMode.PercentOutput, m_joystick.getY());
    m_motor3.set(ControlMode.PercentOutput, m_joystick.getY());
    m_motor4.set(ControlMode.PercentOutput, m_joystick.getY());
    // System.out.println(m_joystick.getY());
    m_motor5.set(ControlMode.PercentOutput, m_joystick.getY());
  }
}
