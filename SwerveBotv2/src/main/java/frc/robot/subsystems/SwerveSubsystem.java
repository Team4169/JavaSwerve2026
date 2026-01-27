package frc.robot.subsystems;

import swervelib.SwerveDrive;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import edu.wpi.first.math.geometry.Translation2d;

public class SwerveSubsystem extends SubsystemBase {

  private final SwerveDrive swerveDrive;

  public SwerveSubsystem(SwerveDrive swerveDrive) {
    this.swerveDrive = swerveDrive;
  }

  public void drive(Translation2d translation,
                    double rotation,
                    boolean fieldRelative,
                    boolean openLoop) {
    swerveDrive.drive(translation, rotation, fieldRelative, openLoop);
  }
}
