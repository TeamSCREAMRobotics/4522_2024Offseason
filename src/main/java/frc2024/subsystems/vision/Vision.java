package frc2024.subsystems.vision;

import com.SCREAMLib.util.ScreamUtil;
import com.SCREAMLib.vision.LimelightHelpers.PoseEstimate;
import com.SCREAMLib.vision.LimelightHelpers.RawFiducial;
import com.SCREAMLib.vision.LimelightVision;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Timer;
import frc2024.constants.FieldConstants;
import java.util.Optional;

public class Vision extends LimelightVision {

  public static final Limelight SHOOTER_LIMELIGHT =
      new Limelight(
          "limelight-shooter",
          new Pose3d(
              0.286, -0.162, 0.233, new Rotation3d(0, Math.toRadians(27), Math.toRadians(180.0))));

  public static final Limelight NOTE_LIMELIGHT =
      new Limelight(
          "limelight-note",
          new Pose3d(0.0, 0.168791, 0.619, new Rotation3d(0.0, -Math.toRadians(1019.987297), 0.0)));

  public static PoseEstimate getRandomPoseEstimate() {
    return new PoseEstimate(
        new Pose2d(
            ScreamUtil.random(0, FieldConstants.FIELD_DIMENSIONS.getX()),
            ScreamUtil.random(0, FieldConstants.FIELD_DIMENSIONS.getY()),
            new Rotation2d()),
        Timer.getFPGATimestamp(),
        ScreamUtil.random(0, 75),
        (int) ScreamUtil.random(0, 4),
        0.0,
        ScreamUtil.random(0, 6),
        ScreamUtil.random(0, 1),
        new RawFiducial[] {});
  }

  public static Optional<Translation2d> getVisibleNotePose(Pose2d robotPose) {
    if (getTV(NOTE_LIMELIGHT)) {
      return Optional.of(
          robotPose
              .getTranslation()
              .plus(
                  new Translation2d(
                      getDistanceToTargetTYBased(FieldConstants.NOTE_HEIGHT, NOTE_LIMELIGHT)
                          .getMeters(),
                      robotPose.getRotation().plus(getAngleToTargetTXBased(NOTE_LIMELIGHT)))));
    } else {
      return Optional.empty();
    }
  }
}
