package frc.robot;

import com.revrobotics.spark.SparkFlex;
import com.revrobotics.spark.SparkLowLevel.MotorType;
import com.revrobotics.spark.SparkMax;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.drive.DifferentialDrive;

/**
 * FRC Team 4169 – 2026 REBUILT Autonomous Routine
 *
 * Plain Java helper class (not a WPILib Command) designed for use with
 * TimedRobot. Robot.java calls init() once in autonomousInit(), then
 * calls update() every loop in autonomousPeriodic().
 *
 * Auto Sequence (start center, shooter already facing Hub):
 *   Phase 1 – FOLDING_DOWN : Fold intake down while stationary (0.4s)
 *   Phase 2 – DRIVING      : Drive straight backward until intake contacts climber area
 *   Phase 3 – SHOOTING     : Stop, run full shoot sequence for 2.5s
 *   Phase 4 – DONE         : Everything stops
 *
 * ── Tuning ────────────────────────────────────────────────────────────────
 *   DRIVE_SPEED  – how fast to drive back (0.0–1.0). Start at 0.4.
 *   FOLD_TIME    – seconds to fold intake before driving. Default 0.4s.
 *   DRIVE_TIME   – seconds to drive back. TUNE THIS on the field until
 *                  intake reliably contacts the climber area.
 *   SHOOT_TIME   – seconds to run shooter. 2.5s is safe for 8 balls.
 */
public class Auto {

    // ── Tuning constants ──────────────────────────────────────────────────
    private static final double DRIVE_SPEED = 0.4;  // backward drive power
    private static final double FOLD_TIME   = 0.4;  // seconds to fold intake down
    private static final double DRIVE_TIME  = 2.0;  // seconds to drive back — TUNE ME
    private static final double SHOOT_TIME  = 2.5;  // seconds to shoot

    // ── Auto phases ───────────────────────────────────────────────────────
    private enum Phase { FOLDING_DOWN, DRIVING, SHOOTING, DONE }
    private Phase m_phase;
    private final Timer m_timer = new Timer();

    // ── Drive ─────────────────────────────────────────────────────────────
    private final SparkMax m_frontLeftMotor;
    private final SparkMax m_frontRightMotor;
    private final SparkMax m_backLeftMotor;
    private final SparkMax m_backRightMotor;
    private final DifferentialDrive m_frontDrive;
    private final DifferentialDrive m_backDrive;

    // ── Mechanisms ────────────────────────────────────────────────────────
    private final SparkMax m_intakeMotor;
    private final SparkMax m_intakeFoldMotor;
    private final SparkFlex m_shooterMotor;
    private final SparkFlex m_kickerMainMotor;
    private final SparkMax m_kickerAuxMotor;

    // ─────────────────────────────────────────────────────────────────────
    /**
     * Pass in all motors from Robot.java so we share the same instances
     * and avoid CAN conflicts.
     */
    public Auto(
            SparkMax frontLeftMotor,
            SparkMax frontRightMotor,
            SparkMax backLeftMotor,
            SparkMax backRightMotor,
            DifferentialDrive frontDrive,
            DifferentialDrive backDrive,
            SparkMax intakeMotor,
            SparkMax intakeFoldMotor,
            SparkFlex shooterMotor,
            SparkFlex kickerMainMotor,
            SparkMax kickerAuxMotor) {

        m_frontLeftMotor  = frontLeftMotor;
        m_frontRightMotor = frontRightMotor;
        m_backLeftMotor   = backLeftMotor;
        m_backRightMotor  = backRightMotor;
        m_frontDrive      = frontDrive;
        m_backDrive       = backDrive;
        m_intakeMotor     = intakeMotor;
        m_intakeFoldMotor = intakeFoldMotor;
        m_shooterMotor    = shooterMotor;
        m_kickerMainMotor = kickerMainMotor;
        m_kickerAuxMotor  = kickerAuxMotor;
    }

    // ─────────────────────────────────────────────────────────────────────
    /** Call once from Robot.autonomousInit() */
    public void init() {
        m_phase = Phase.FOLDING_DOWN;
        m_timer.reset();
        m_timer.start();
        stopAll();

        // Start folding intake down immediately.
        m_intakeFoldMotor.set(0.1);
    }

    // ─────────────────────────────────────────────────────────────────────
    /** Call every loop from Robot.autonomousPeriodic() */
    public void update() {
        switch (m_phase) {

            case FOLDING_DOWN:
                // Fold intake down, stay still.
                m_intakeFoldMotor.set(0.1);
                if (m_timer.hasElapsed(FOLD_TIME)) {
                    m_intakeFoldMotor.stopMotor();
                    m_timer.reset();
                    m_phase = Phase.DRIVING;
                    // Start driving backward.
                    m_frontDrive.tankDrive(-DRIVE_SPEED, -DRIVE_SPEED);
                    m_backDrive.tankDrive(-DRIVE_SPEED, -DRIVE_SPEED);
                }
                break;

            case DRIVING:
                // Keep driving backward.
                m_frontDrive.tankDrive(-DRIVE_SPEED, -DRIVE_SPEED);
                m_backDrive.tankDrive(-DRIVE_SPEED, -DRIVE_SPEED);
                if (m_timer.hasElapsed(DRIVE_TIME)) {
                    // Intake has hit the climber area — stop and shoot.
                    m_frontDrive.tankDrive(0, 0);
                    m_backDrive.tankDrive(0, 0);
                    m_timer.reset();
                    m_phase = Phase.SHOOTING;
                    // Start full shoot sequence (same values as teleop Y button).
                    m_shooterMotor.set(-0.51);
                    m_kickerMainMotor.set(0.5);
                    m_kickerAuxMotor.set(-0.44);
                    m_intakeMotor.set(-0.8);
                }
                break;

            case SHOOTING:
                // Hold shoot sequence until time is up.
                if (m_timer.hasElapsed(SHOOT_TIME)) {
                    stopAll();
                    m_phase = Phase.DONE;
                }
                break;

            case DONE:
                // Nothing to do — robot sits still.
                break;
        }
    }

    // ─────────────────────────────────────────────────────────────────────
    /** Call from Robot.autonomousExit() or if auto is interrupted. */
    public void stop() {
        stopAll();
    }

    // ─────────────────────────────────────────────────────────────────────
    private void stopAll() {
        m_frontDrive.tankDrive(0, 0);
        m_backDrive.tankDrive(0, 0);
        m_intakeMotor.stopMotor();
        m_intakeFoldMotor.stopMotor();
        m_shooterMotor.stopMotor();
        m_kickerMainMotor.stopMotor();
        m_kickerAuxMotor.stopMotor();
    }
}
