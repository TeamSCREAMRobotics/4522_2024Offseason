package frc2025.logging;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import frc2025.subsystems.elevator.ElevatorConstants;

public class ComponentConstants {
  public static final Translation3d ELEV_STAGE1_POSE = new Translation3d(0.03105, 0.0, 0.171209);
  public static final Translation3d ELEV_STAGE2_POSE = new Translation3d(0.0355, 0.0, 0.196223);
  public static final Translation3d SHOOTER_POSE = new Translation3d(0.074840, 0.0, 0.418531);
  public static final Translation3d STABLIZER_BARS_POSE =
      new Translation3d(0.261421, 0.0, 0.195021);

  private static final double SIN_80 = Math.sin(Math.toRadians(80));
  private static final double COS_80 = Math.cos(Math.toRadians(80));

  private static final double ELEV_STAGE_DISTANCE = 0.025; // Approximate distance between stages

  private static final double SHOOTER_OFFSET_X = 0.225918 * COS_80;
  private static final double SHOOTER_OFFSET_Z = 0.225918 * SIN_80;

  public static Pose3d getElevStage1Pose(double elevatorHeight) {
    double clampedHeight = MathUtil.clamp(elevatorHeight, 0, ElevatorConstants.MAX_HEIGHT);
    double offsetX = clampedHeight * COS_80;
    double offsetZ = clampedHeight * SIN_80;

    double stage1OffsetX, stage1OffsetZ;
    if (elevatorHeight <= Units.inchesToMeters(10.875) - ELEV_STAGE_DISTANCE) {
      stage1OffsetX = -(elevatorHeight + ELEV_STAGE_DISTANCE) * COS_80;
      stage1OffsetZ = -(elevatorHeight + ELEV_STAGE_DISTANCE) * SIN_80;
    } else {
      stage1OffsetX = -(Units.inchesToMeters(11.875) - ELEV_STAGE_DISTANCE) * COS_80;
      stage1OffsetZ = -(Units.inchesToMeters(11.875) - ELEV_STAGE_DISTANCE) * SIN_80;
    }

    return new Pose3d(
        ELEV_STAGE2_POSE.getX() + offsetX + stage1OffsetX,
        ELEV_STAGE2_POSE.getY(),
        ELEV_STAGE2_POSE.getZ() + offsetZ + stage1OffsetZ,
        Rotation3d.kZero);
  }

  public static Pose3d getElevStage2Pose(double elevatorHeight) {
    double clampedHeight = MathUtil.clamp(elevatorHeight, 0, ElevatorConstants.MAX_HEIGHT);
    double offsetX = clampedHeight * COS_80;
    double offsetZ = clampedHeight * SIN_80;
    return new Pose3d(
        ELEV_STAGE2_POSE.getX() + offsetX,
        ELEV_STAGE2_POSE.getY(),
        ELEV_STAGE2_POSE.getZ() + offsetZ,
        Rotation3d.kZero);
  }

  public static Pose3d getShooterPose(double elevatorHeight, Rotation2d pivotAngle) {
    Pose3d elevStage2Pose = getElevStage2Pose(elevatorHeight);
    double pivotRad = pivotAngle.getRadians();

    return new Pose3d(
        elevStage2Pose.getX() + SHOOTER_OFFSET_X,
        elevStage2Pose.getY(),
        elevStage2Pose.getZ() + SHOOTER_OFFSET_Z,
        new Rotation3d(0, pivotRad, 0));
  }

  public static Pose3d getStabilizerPose(Rotation2d stabilizerAngle) {
    return new Pose3d(
        STABLIZER_BARS_POSE.getX(),
        STABLIZER_BARS_POSE.getY(),
        STABLIZER_BARS_POSE.getZ(),
        new Rotation3d(0, stabilizerAngle.getRadians(), 0));
  }
}
