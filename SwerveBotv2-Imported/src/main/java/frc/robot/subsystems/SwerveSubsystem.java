package frc.robot.subsystems;

import swervelib.SwerveDrive;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.geometry.Translation2d;

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

    public SwerveSubsystem(SwerveDrive swerveDrive) {
        this.swerveDrive = swerveDrive;

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
    }

    @Override
    public void periodic() {
        poseEstimator.update(
            swerveDrive.getYaw(),
            swerveDrive.getModulePositions()
        );

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
}

