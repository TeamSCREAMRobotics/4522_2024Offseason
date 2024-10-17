package frc2025.subsystems.vision;

import com.SCREAMLib.data.Length;
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
import frc2025.constants.FieldConstants;
import java.util.Optional;

public class Vision extends LimelightVision {

  public static final Translation2d L3G_FOV = new Translation2d(82, 56.2);

  public static final Limelight SHOOTER_LIMELIGHT =
      new Limelight(
          "limelight-shooter",
          new Pose3d(
              0.286, -0.162, 0.233, new Rotation3d(0, Math.toRadians(27), Math.toRadians(180.0))));
  public static final Limelight NOTE_LIMELIGHT =
      new Limelight(
          "limelight-note",
          new Pose3d(0.0, 0.168791, 0.619, new Rotation3d(0.0, -Math.toRadians(28), 0.0)));

  public static PoseEstimate getRandomPoseEstimate() {
    return new PoseEstimate(
        new Pose2d(
            ScreamUtil.random(0, FieldConstants.FIELD_DIMENSIONS.getX()),
            ScreamUtil.random(0, FieldConstants.FIELD_DIMENSIONS.getY()),
            Rotation2d.kZero),
        Timer.getFPGATimestamp(),
        ScreamUtil.random(0, 75),
        (int) ScreamUtil.random(0, 4),
        0.0,
        ScreamUtil.random(0, 6),
        ScreamUtil.random(0, 1),
        new RawFiducial[] {});
  }

  public static Optional<Translation2d> getVisibleNotePose(Pose2d robotPose) {
    Length distance = getDistanceToTargetTYBased(FieldConstants.NOTE_HEIGHT, NOTE_LIMELIGHT);
    if (getTV(NOTE_LIMELIGHT)) {
      return Optional.of(
          robotPose
              .getTranslation()
              .plus(new Translation2d(NOTE_LIMELIGHT.relativePosition().getY(), 0))
              .plus(
                  new Translation2d(
                      distance.getMeters(),
                      robotPose.getRotation().plus(getAngleToTargetTXBased(NOTE_LIMELIGHT)))));
    } else {
      return Optional.empty();
    }
  }

  public static void periodic() {
    if (getTV(SHOOTER_LIMELIGHT)) {
      double tx = getTX(SHOOTER_LIMELIGHT) / (L3G_FOV.getX() / 2.0);
      double ty = getTY(SHOOTER_LIMELIGHT) / (L3G_FOV.getX() / 2.0);
      setCropWindow(tx - 0.5, tx + 0.5, ty - 0.3, ty + 0.3, SHOOTER_LIMELIGHT);
    } else {
      setCropWindow(-1, 1, -1, 1, SHOOTER_LIMELIGHT);
    }
  }
}
