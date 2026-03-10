package frc.robot.subsystems;

import swervelib.SwerveDrive;
import com.studica.frc.AHRS;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import frc.robot.Vision;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import swervelib.telemetry.SwerveDriveTelemetry;
import swervelib.telemetry.SwerveDriveTelemetry.TelemetryVerbosity;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class SwerveSubsystem extends SubsystemBase {

  private final SwerveDrive swerveDrive;
  private final SwerveDrivePoseEstimator poseEstimator;
  private final Vision vision;
  private final Field2d field = new Field2d();
  private final AHRS navx;

  private final Timer startupSyncTimer = new Timer();
  private boolean startupSyncDone = false;

  public SwerveSubsystem(SwerveDrive swerveDrive) {
    this.swerveDrive = swerveDrive;
    SwerveDriveTelemetry.verbosity = TelemetryVerbosity.LOW;

    // Keep absolute calibration from being reapplied periodically while driving.
    swerveDrive.setModuleEncoderAutoSynchronize(false, 1);
    SmartDashboard.putBoolean("Swerve/AutoSyncEnabled", false);

    startupSyncTimer.start();
    SmartDashboard.putBoolean("Swerve/StartupSyncDone", false);

    Object imu = swerveDrive.getGyro().getIMU();
    navx = imu instanceof AHRS ? (AHRS) imu : null;
    SmartDashboard.putBoolean("NavX Detected", navx != null);

    poseEstimator = new SwerveDrivePoseEstimator(
        swerveDrive.kinematics,
        swerveDrive.getYaw(),
        swerveDrive.getModulePositions(),
        new Pose2d(8.25, 1.25, new Rotation2d()));

    vision = new Vision((pose, timestamp, stdDevs) -> poseEstimator.addVisionMeasurement(pose, timestamp, stdDevs));
    SmartDashboard.putData("Field", field);
  }

  @Override
  public void periodic() {
    if (!startupSyncDone && startupSyncTimer.hasElapsed(1.5)) {
      synchronizeToAbsoluteEncoders();
      startupSyncDone = true;
      SmartDashboard.putBoolean("Swerve/StartupSyncDone", true);
    }

    poseEstimator.update(swerveDrive.getYaw(), swerveDrive.getModulePositions());
    field.setRobotPose(poseEstimator.getEstimatedPosition());

    Pose2d currentPose = poseEstimator.getEstimatedPosition();
    SmartDashboard.putNumber("Heading (deg)", swerveDrive.getYaw().getDegrees());
    SmartDashboard.putNumber("Pose/X", currentPose.getX());
    SmartDashboard.putNumber("Pose/Y", currentPose.getY());
    SmartDashboard.putNumber("Pose/Angle", currentPose.getRotation().getDegrees());

    ChassisSpeeds speeds = getSpeeds();
    SmartDashboard.putNumber("Chassis/VelocityX", speeds.vxMetersPerSecond);
    SmartDashboard.putNumber("Chassis/VelocityY", speeds.vyMetersPerSecond);
    SmartDashboard.putNumber("Chassis/AngularVelocity", speeds.omegaRadiansPerSecond);

    SwerveModuleState[] moduleStates = swerveDrive.getStates();
    String[] moduleNames = {"FrontLeft", "FrontRight", "BackLeft", "BackRight"};
    for (int i = 0; i < moduleStates.length && i < moduleNames.length; i++) {
      String moduleName = moduleNames[i];
      SmartDashboard.putNumber("Module/" + moduleName + "/Velocity", moduleStates[i].speedMetersPerSecond);
      SmartDashboard.putNumber("Module/" + moduleName + "/Angle", moduleStates[i].angle.getDegrees());
    }

    var modulePositions = swerveDrive.getModulePositions();
    for (int i = 0; i < modulePositions.length && i < moduleNames.length; i++) {
      String moduleName = moduleNames[i];
      SmartDashboard.putNumber("ModulePos/" + moduleName + "/Distance", modulePositions[i].distanceMeters);
      SmartDashboard.putNumber("ModulePos/" + moduleName + "/Angle", modulePositions[i].angle.getDegrees());
    }

    if (navx != null) {
      SmartDashboard.putBoolean("NavX Connected", navx.isConnected());
      SmartDashboard.putBoolean("NavX Calibrating", navx.isCalibrating());
      SmartDashboard.putNumber("NavX Yaw (deg)", navx.getYaw());
    }
    vision.periodic();
  }

  public void drive(Translation2d translation, double rotation, boolean fieldRelative, boolean openLoop) {
    swerveDrive.drive(translation, rotation, fieldRelative, openLoop);
  }

  public Pose2d getPose() {
    return poseEstimator.getEstimatedPosition();
  }

  public void resetPose(Pose2d pose) {
    poseEstimator.resetPosition(swerveDrive.getYaw(), swerveDrive.getModulePositions(), pose);
  }

  public void zeroHeading() {
    swerveDrive.zeroGyro();
    Pose2d current = getPose();
    resetPose(new Pose2d(current.getTranslation(), Rotation2d.kZero));
  }

  public void initializeToDefaultPositions() {
    if (!DriverStation.isDisabled()) {
      SmartDashboard.putString("Swerve/SyncStatus", "Ignored (not disabled)");
      return;
    }
    swerveDrive.resetDriveEncoders();
    synchronizeToAbsoluteEncoders();
    zeroHeading();
    SmartDashboard.putString("Swerve/SyncStatus", "Applied");
  }

  public void synchronizeToAbsoluteEncoders() {
    swerveDrive.synchronizeModuleEncoders();
    SmartDashboard.putString("Swerve/LastSync", "Applied");
  }

  public void pointModulesForward() {
    SwerveModuleState[] forwardStates = new SwerveModuleState[swerveDrive.getStates().length];
    for (int i = 0; i < forwardStates.length; i++) {
      forwardStates[i] = new SwerveModuleState(0.0, Rotation2d.kZero);
    }
    swerveDrive.setModuleStates(forwardStates, false);
  }

  public boolean isNavXConnected() {
    return navx != null && navx.isConnected();
  }

  public ChassisSpeeds getSpeeds() {
    return swerveDrive.getRobotVelocity();
  }

  public void driveRobotRelative(ChassisSpeeds speeds) {
    swerveDrive.drive(speeds);
  }
}
