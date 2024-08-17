package frc2024.logging;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;

public class ComponentConstants {
  public static final Pose3d ELEV_STAGE1_POSE =
      new Pose3d(new Translation3d(0.03105, 0.0, 0.171209), new Rotation3d());
  public static final Pose3d ELEV_STAGE2_POSE =
      new Pose3d(new Translation3d(0.0355, 0.0, 0.196223), new Rotation3d());
  public static final Pose3d SHOOTER_POSE =
      new Pose3d(new Translation3d(0.074840, 0.0, 0.418531), new Rotation3d());
  public static final Pose3d STABLIZER_BARS_POSE =
      new Pose3d(new Translation3d(0.261421, 0.0, 0.195021), new Rotation3d());

  public static Pose3d getElevStage1Pose(double elevatorHeight) {
    double distance =
        ELEV_STAGE2_POSE.getTranslation().getDistance(ELEV_STAGE1_POSE.getTranslation());
    double offset =
        (elevatorHeight <= Units.inchesToMeters(10.875) - distance)
            ? -elevatorHeight - distance
            : -Units.inchesToMeters(11.875) + distance;

    Translation2d t2d = new Translation2d(offset, Rotation2d.fromDegrees(80));
    Translation3d t3d = new Translation3d(t2d.getX(), 0, t2d.getY());

    return getElevStage2Pose(elevatorHeight).plus(new Transform3d(t3d, new Rotation3d()));
  }

  public static Pose3d getElevStage2Pose(double elevatorHeight) {
    Translation2d x =
        new Translation2d(
            MathUtil.clamp(elevatorHeight, 0, Units.inchesToMeters(21.75)),
            Rotation2d.fromDegrees(80));
    Translation3d y = new Translation3d(x.getX(), 0, x.getY());
    return ELEV_STAGE2_POSE.plus(new Transform3d(y, new Rotation3d()));
  }

  public static Pose3d getShooterPose(double elevatorHeight, Rotation2d pivotAngle) {
    Translation2d x = new Translation2d(0.225918, Rotation2d.fromDegrees(80));
    Translation3d y = new Translation3d(x.getX(), 0, x.getY());
    return getElevStage2Pose(elevatorHeight)
        .plus(new Transform3d(y, new Rotation3d(0, pivotAngle.getRadians(), 0)));
  }

  public static Pose3d getStabilizerPose(Rotation2d stabilizerAngle) {
    return STABLIZER_BARS_POSE.transformBy(
        new Transform3d(new Translation3d(), new Rotation3d(0, stabilizerAngle.getRadians(), 0)));
  }
}
