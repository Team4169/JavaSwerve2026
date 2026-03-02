package frc.robot.subsystems;

import swervelib.SwerveDrive;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.math.geometry.Pose2d; 
import frc.robot.Vision;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import swervelib.telemetry.SwerveDriveTelemetry;
import swervelib.telemetry.SwerveDriveTelemetry.TelemetryVerbosity;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;


//public class SwerveSubsystem extends SubsystemBase {

//  private final SwerveDrive swerveDrive;

//  public SwerveSubsystem(SwerveDrive swerveDrive) {
//    this.swerveDrive = swerveDrive;
//  }

//  public void drive(Translation2d translation,
//                    double rotation,
//                    boolean fieldRelative,
//                    boolean openLoop) {
//    swerveDrive.drive(translation, rotation, fieldRelative, openLoop);
//  }
//}
public class SwerveSubsystem extends SubsystemBase {

    private final SwerveDrive swerveDrive;
    private final SwerveDrivePoseEstimator poseEstimator;
    private final Vision vision;
    private final Field2d field = new Field2d();


    public SwerveSubsystem(SwerveDrive swerveDrive) {
        this.swerveDrive = swerveDrive;
        SwerveDriveTelemetry.verbosity = TelemetryVerbosity.HIGH; //WE need to comment ts out for competition
        poseEstimator = new SwerveDrivePoseEstimator(
            swerveDrive.kinematics,
            swerveDrive.getYaw(),
            swerveDrive.getModulePositions(),
            new Pose2d()
        );

        vision = new Vision(
            (pose, timestamp, stdDevs) -> {
                poseEstimator.addVisionMeasurement(pose, timestamp, stdDevs);
            }
        );
        SmartDashboard.putData("Field", field);
    }

    @Override
    public void periodic() {
        poseEstimator.update(
            swerveDrive.getYaw(),
            swerveDrive.getModulePositions()
        );
        field.setRobotPose(poseEstimator.getEstimatedPosition());
        vision.periodic();
    }

    public void drive(Translation2d translation,
                      double rotation,
                      boolean fieldRelative,
                      boolean openLoop) {
        swerveDrive.drive(translation, rotation, fieldRelative, openLoop);
    }

    public Pose2d getPose() {
        return poseEstimator.getEstimatedPosition();
    }

    public void resetPose(Pose2d pose) {
        poseEstimator.resetPosition(
            swerveDrive.getYaw(),
            swerveDrive.getModulePositions(),
            pose
        );
    }

    public ChassisSpeeds getSpeeds() { 
        return swerveDrive.getRobotVelocity(); 
    } 
    
    public void driveRobotRelative(ChassisSpeeds speeds) { 
        swerveDrive.drive(speeds); 
    }
}

