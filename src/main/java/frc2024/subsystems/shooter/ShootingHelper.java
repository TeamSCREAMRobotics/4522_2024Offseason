package frc2024.subsystems.shooter;

import com.SCREAMLib.data.DataConversions;
import com.SCREAMLib.math.ScreamMath;
import com.SCREAMLib.util.AllianceFlipUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.DriverStation;
import frc2024.RobotContainer;
import frc2024.RobotContainer.Subsystems;
import frc2024.RobotState;
import frc2024.constants.FieldConstants;
import frc2024.subsystems.pivot.Pivot.PivotGoal;
import frc2024.subsystems.pivot.PivotConstants;
import frc2024.subsystems.shooter.ShootStateInterpolatingTreeMap.ShootState;

public class ShootingHelper {
  public record ShotParameters(
      Rotation2d targetHeading,
      ShootState shootState,
      double effectiveDistance,
      double actualDistance) {}

  public enum Zone {
    ALLIANCE_WING,
    CENTER,
    OPPOSING_WING;
  }

  private static final Subsystems subsystems = RobotContainer.getSubsystems();

  public static Translation3d[] calculateSimpleTrajectory(Pose2d pose, double horizontalDistance) {
    Translation2d shooterExitPos =
        RobotState.getPivotRootPosition()
            .get()
            .plus(
                new Translation2d(
                    PivotConstants.SHOOTER_DISTANCE_FROM_AXLE.getMeters(),
                    subsystems.pivot().getAngle().unaryMinus().plus(Rotation2d.fromDegrees(90))))
            .plus(
                new Translation2d(
                    ShooterConstants.SHOOTER_BACK_LENGTH.getMeters(),
                    subsystems.pivot().getAngle().unaryMinus().plus(new Rotation2d(Math.PI))));

    Translation2d endPoint2d =
        shooterExitPos.plus(
            new Translation2d(
                horizontalDistance,
                subsystems.pivot().getAngle().unaryMinus().plus(new Rotation2d(Math.PI))));

    Translation3d startPoint =
        ScreamMath.rotatePoint(
                new Translation3d(shooterExitPos.getX(), 0, shooterExitPos.getY()),
                pose.getRotation())
            .plus(new Translation3d(pose.getX(), pose.getY(), 0));
    Translation3d endPoint =
        ScreamMath.rotatePoint(
                new Translation3d(endPoint2d.getX(), 0, endPoint2d.getY()), pose.getRotation())
            .plus(new Translation3d(pose.getX(), pose.getY(), 0));

    return new Translation3d[] {startPoint, endPoint};
  }

  public static ShotParameters calculateSimpleShotParameters(
      Translation2d currentTranslation, Translation2d targetTranslation) {
    Translation2d shotOffset =
        getMoveWhileShootOffset(subsystems.drivetrain().getFieldRelativeSpeeds());
    double actualDistance = currentTranslation.getDistance(targetTranslation);
    double effectiveDistance = actualDistance + shotOffset.getX();
    ShootState mapped = ShooterConstants.SHOOTING_MAP.get(effectiveDistance);
    ShootState calculated = new ShootState();
    Zone currentZone = getZone(currentTranslation);

    switch (currentZone) {
      case CENTER:
        handleCenterZone(calculated, effectiveDistance, mapped);
        if (!DriverStation.isAutonomous())
          targetTranslation =
              AllianceFlipUtil.MirroredTranslation2d(
                  FieldConstants.PointsOfInterest.BLUE_WING_FEED_TRANSLATION);
        break;
      case OPPOSING_WING:
        handleOpposingWingZone(calculated);
        if (!DriverStation.isAutonomous())
          targetTranslation =
              AllianceFlipUtil.MirroredTranslation2d(
                  FieldConstants.PointsOfInterest.BLUE_CENTER_FEED_TRANSLATION);
        break;
      default:
        handleDefaultZone(calculated, effectiveDistance, mapped);
    }

    calculated.setVelocityRPM(calculateVelocityRPM(mapped));

    Rotation2d targetHeading =
        calculateTargetHeading(targetTranslation, currentTranslation, shotOffset);

    return new ShotParameters(targetHeading, calculated, effectiveDistance, actualDistance);
  }

  private static void handleCenterZone(
      ShootState calculated, double effectiveDistance, ShootState mapped) {
    if (DriverStation.isAutonomous() && !subsystems.conveyor().hasNote().getAsBoolean()) {
      calculated.setPivotAngle(PivotGoal.HOME_INTAKE.getAngle());
    } else if (!withinRange()) {
      calculated.setPivotAngle(PivotGoal.FEED_TO_WING.getAngle());
    } else {
      setPivotAngleAndElevatorHeight(calculated, effectiveDistance, mapped);
    }
  }

  private static void handleOpposingWingZone(ShootState calculated) {
    calculated.setPivotAngle(PivotGoal.FEED_TO_CENTER.getAngle());
  }

  private static void handleDefaultZone(
      ShootState calculated, double effectiveDistance, ShootState mapped) {
    setPivotAngleAndElevatorHeight(calculated, effectiveDistance, mapped);
  }

  private static void setPivotAngleAndElevatorHeight(
      ShootState calculated, double effectiveDistance, ShootState mapped) {
    calculated.setPivotAngle(
        getPivotAngleToGoal(
            effectiveDistance, FieldConstants.ScoringLocations.SPEAKER_OPENING.getZ()));
    calculated.setElevatorHeight(mapped.elevatorHeight);
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

  private static Rotation2d getPivotAngleToGoal(double horizontalDistance, double goalHeight) {
    return ScreamMath.clamp(
        ScreamMath.calculateAngleToPoint(
            RobotState.getPivotRootPosition().get(),
            new Translation2d(
                horizontalDistance,
                goalHeight - PivotConstants.SHOOTER_DISTANCE_FROM_AXLE.getMeters())),
        Rotation2d.fromRotations(PivotGoal.SUB.getTarget().getAsDouble()),
        Rotation2d.fromDegrees(
            subsystems.elevator().getMeasuredHeight().getInches() > 1.5 ? 0 : 4));
  }

  public static Translation2d getMoveWhileShootOffset(ChassisSpeeds robotSpeeds) {
    double[] temp = DataConversions.chassisSpeedsToArray(robotSpeeds);
    return new Translation2d(
        temp[0] / 4.0, temp[1] * AllianceFlipUtil.getDirectionCoefficient() / 3.0);
  }

  private static boolean pointedAwayFromGoal() {
    return subsystems
        .drivetrain()
        .getWithinAngleThreshold(
            AllianceFlipUtil.MirroredRotation2d(Rotation2d.fromDegrees(180)),
            Rotation2d.fromDegrees(90));
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
        && ScreamMath.getLinearVelocity(subsystems.drivetrain().getRobotRelativeSpeeds()) < 2.5
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
