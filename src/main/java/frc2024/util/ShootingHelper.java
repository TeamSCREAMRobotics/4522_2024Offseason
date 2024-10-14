package frc2024.util;

import com.SCREAMLib.data.DataConversions;
import com.SCREAMLib.math.ScreamMath;
import com.SCREAMLib.util.AllianceFlipUtil;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import frc2024.RobotContainer;
import frc2024.RobotContainer.Subsystems;
import frc2024.RobotState;
import frc2024.constants.FieldConstants;
import frc2024.subsystems.pivot.Pivot.PivotGoal;
import frc2024.subsystems.shooter.ShooterConstants;
import frc2024.util.ShootStateInterpolatingTreeMap.ShootState;

public class ShootingHelper {
  public record ShotParameters(
      Rotation2d targetHeading,
      ShootState shootState,
      double effectiveDistance,
      double actualDistance,
      Translation2d shotLookahead) {}

  public enum Zone {
    ALLIANCE_WING,
    CENTER,
    OPPOSING_WING;
  }

  private static final Subsystems subsystems = RobotContainer.getSubsystems();

  public static ShotParameters calculateSimpleShotParameters(
      Translation2d currentTranslation, Translation2d targetTranslation) {
    double actualDistance = currentTranslation.getDistance(targetTranslation);
    Translation2d shotLookahead =
        getShotLookahead(subsystems.drivetrain().getFieldRelativeSpeeds(), actualDistance);
    double effectiveDistance = actualDistance + shotLookahead.getX();
    ShootState mapped = ShooterConstants.SHOOTING_MAP.get(effectiveDistance);
    ShootState calculated = new ShootState();

    setPivotAngleAndElevatorHeight(calculated, effectiveDistance, mapped);
    calculated.setVelocityRPM(calculateVelocityRPM(mapped));

    Rotation2d targetHeading =
        calculateTargetHeading(targetTranslation, currentTranslation, shotLookahead);

    return new ShotParameters(
        targetHeading, calculated, effectiveDistance, actualDistance, shotLookahead);
  }

  private static void setPivotAngleAndElevatorHeight(
      ShootState calculated, double effectiveDistance, ShootState mapped) {
    calculated.setPivotAngle(getPivotAngleToGoal(mapped, effectiveDistance));
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

  private static Rotation2d getPivotAngleToGoal(ShootState mapped, double horizontalDistance) {
    return ScreamMath.clamp(
        mapped.getPivotAngle(),
        Rotation2d.fromRotations(PivotGoal.SUB.getTarget().getAsDouble()),
        Rotation2d.fromDegrees(
            subsystems.elevator().getMeasuredHeight().getInches() > 1.5 ? 0 : 4));
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

  public static boolean pointedAtGoal(double horizontalDistance) {
    Translation2d translation =
        subsystems
            .drivetrain()
            .getPose()
            .getTranslation()
            .minus(new Translation2d(horizontalDistance, subsystems.drivetrain().getHeading()));
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

  public static boolean validShot(double horizontalDistance) {
    return shooterAtSpeed(horizontalDistance)
        && subsystems.elevator().atGoal()
        && subsystems.pivot().atGoal()
        && pointedAtGoal(horizontalDistance)
        && subsystems.drivetrain().getLinearVelocity() < 2.5
        && withinRange();
  }

  private static boolean shooterAtSpeed(double horizontalDistance) {
    if (horizontalDistance < 2.5) {
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
