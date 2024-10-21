package frc2025.subsystems.vision;

import data.Length;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Timer;
import frc2025.constants.FieldConstants;
import java.util.Optional;
import util.ScreamUtil;
import vision.LimelightHelpers.PoseEstimate;
import vision.LimelightHelpers.RawFiducial;
import vision.LimelightVision;

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

  public static Translation2d lastNotePose;

  public static final Timer lastNoteTimer = new Timer();

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

  public static Optional<Translation2d> getNotePose(Pose2d robotPose) {
    if (getTV(NOTE_LIMELIGHT)) {
      Length distance = getDistanceToTargetTYBased(FieldConstants.NOTE_HEIGHT, NOTE_LIMELIGHT);
      Translation2d pose =
          robotPose
              .getTranslation()
              .plus(new Translation2d(NOTE_LIMELIGHT.relativePosition().getY(), 0))
              .plus(
                  new Translation2d(
                      distance.getMeters(),
                      robotPose.getRotation().plus(getAngleToTargetTXBased(NOTE_LIMELIGHT))));
      if (lastNotePose != Translation2d.kZero
          && lastNotePose.getDistance(pose) > 0.1
          && lastNoteTimer.get() == 0.0) {
        lastNoteTimer.start();
        return Optional.of(lastNotePose);
      }
      if (lastNoteTimer.get() > 0.0 && lastNoteTimer.get() < 1.0) {
        return Optional.of(lastNotePose);
      }
      lastNoteTimer.reset();
      lastNotePose = pose;
      return Optional.of(pose);
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
