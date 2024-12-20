package frc2025.util;

import data.DataConversions;
import data.Length;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import frc2025.RobotContainer;
import frc2025.RobotContainer.Subsystems;
import frc2025.RobotState;
import frc2025.constants.FieldConstants;
import frc2025.subsystems.pivot.Pivot.PivotGoal;
import frc2025.subsystems.shooter.ShooterConstants;
import frc2025.util.ShootStateInterpolatingTreeMap.ShootState;
import util.AllianceFlipUtil;

public class ShootingHelper {
  public record ShotParameters(
      Rotation2d targetHeading,
      ShootState shootState,
      Length effectiveDistance,
      Length actualDistance,
      Translation2d shotLookahead) {}

  public enum Zone {
    ALLIANCE_WING,
    CENTER,
    OPPOSING_WING;
  }

  private static final Subsystems subsystems = RobotContainer.getSubsystems();

  public static ShotParameters calculateSimpleShotParameters(
      Translation2d currentTranslation, Translation2d targetTranslation) {
    Length actualDistance = Length.fromMeters(currentTranslation.getDistance(targetTranslation));
    Translation2d shotLookahead =
        getShotLookahead(
            subsystems.drivetrain().getFieldRelativeSpeeds(), actualDistance.getMeters());
    Length effectiveDistance = actualDistance.plus(Length.fromMeters(shotLookahead.getX()));
    ShootState mapped = ShooterConstants.SHOOTING_MAP.get(effectiveDistance.getMeters());
    ShootState calculated = new ShootState();

    setPivotAngleAndElevatorHeight(calculated, effectiveDistance.getMeters(), mapped);
    calculated.setVelocityRPM(calculateVelocityRPM(mapped));

    Rotation2d targetHeading =
        calculateTargetHeading(targetTranslation, currentTranslation, shotLookahead);

    return new ShotParameters(
        targetHeading, calculated, effectiveDistance, actualDistance, shotLookahead);
  }

  private static void setPivotAngleAndElevatorHeight(
      ShootState calculated, double effectiveDistance, ShootState mapped) {
    calculated.setPivotAngleDeg(getPivotAngleToGoal(mapped, effectiveDistance));
    if (!DriverStation.isAutonomous()) {
      calculated.setElevatorHeight(mapped.elevatorHeight);
    }
  }

  private static double calculateVelocityRPM(ShootState mapped) {
    return (subsystems.conveyor().hasNote().getAsBoolean() && withinRange())
            || DriverStation.isAutonomous()
        ? mapped.velocityRPM
        : 2000;
  }

  private static Rotation2d calculateTargetHeading(
      Translation2d targetTranslation, Translation2d currentTranslation, Translation2d shotOffset) {
    Translation2d raw = targetTranslation.minus(currentTranslation);
    return raw.minus(new Translation2d(0, shotOffset.getY()))
        .getAngle()
        .minus(new Rotation2d(Math.PI));
  }

  private static double getPivotAngleToGoal(ShootState mapped, double horizontalDistance) {
    return MathUtil.clamp(
        mapped.getPivotAngleDeg(),
        subsystems.elevator().getMeasuredHeight().getInches() > 1.5 ? 0 : 4,
        Units.rotationsToDegrees(PivotGoal.SUB.target().getAsDouble()));
  }

  public static Translation2d getShotLookahead(ChassisSpeeds robotSpeeds, double actualDistance) {
    double[] temp = DataConversions.chassisSpeedsToArray(robotSpeeds);
    return new Translation2d(
        temp[0] / MathUtil.clamp((2.5 * actualDistance) - 4, 4.0, Double.MAX_VALUE),
        temp[1] * AllianceFlipUtil.getDirectionCoefficient() / 3.0);
  }

  public static boolean withinRange() {
    double horizontalDistance =
        RobotContainer.getSubsystems()
            .drivetrain()
            .getPose()
            .getTranslation()
            .getDistance(RobotState.getActiveSpeaker().get().toTranslation2d());
    return horizontalDistance < 8;
  }

  public static boolean pointedAtGoal(Length horizontalDistance) {
    Translation2d translation =
        subsystems
            .drivetrain()
            .getPose()
            .getTranslation()
            .minus(
                new Translation2d(
                    horizontalDistance.getMeters(), subsystems.drivetrain().getHeading()));
    return translation.getY()
            < AllianceFlipUtil.MirroredTranslation3d(
                        FieldConstants.ScoringLocations.SPEAKER_BOTTOM_RIGHT)
                    .getY()
                - FieldConstants.NOTE_DIAMETER.div(2).getMeters()
        && translation.getY()
            > AllianceFlipUtil.MirroredTranslation3d(
                        FieldConstants.ScoringLocations.SPEAKER_BOTTOM_LEFT)
                    .getY()
                + FieldConstants.NOTE_DIAMETER.div(2).getMeters();
  }

  public static boolean validShot(Length horizontalDistance) {
    return shooterAtSpeed(horizontalDistance)
        && subsystems.elevator().atGoal()
        && subsystems.pivot().atGoal()
        && pointedAtGoal(horizontalDistance)
        && subsystems.drivetrain().getLinearVelocity() < 2.5
        && withinRange();
  }

  private static boolean shooterAtSpeed(Length horizontalDistance) {
    if (horizontalDistance.getMeters() < 2.5) {
      return subsystems.shooter().getError() <= 5.0;
    } else {
      return subsystems.shooter().atGoal();
    }
  }

  public static Zone getZone(Translation2d pose) {
    if (AllianceFlipUtil.get(
            FieldConstants.Zones.BLUE_WING_AREA, FieldConstants.Zones.RED_WING_AREA)
        .contains(pose)) {
      return Zone.ALLIANCE_WING;
    } else if (AllianceFlipUtil.get(
            FieldConstants.Zones.RED_WING_AREA, FieldConstants.Zones.BLUE_WING_AREA)
        .contains(pose)) {
      return Zone.OPPOSING_WING;
    } else {
      return Zone.CENTER;
    }
  }
}
