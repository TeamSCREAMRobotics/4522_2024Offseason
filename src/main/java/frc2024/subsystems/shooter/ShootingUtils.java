package frc2024.subsystems.shooter;

import com.SCREAMLib.data.DataConversions;
import com.SCREAMLib.math.ScreamMath;
import com.SCREAMLib.util.AllianceFlipUtil;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import frc2024.RobotContainer;
import frc2024.RobotContainer.Subsystems;
import frc2024.RobotState;
import frc2024.constants.FieldConstants;
import frc2024.subsystems.pivot.Pivot.PivotGoal;
import frc2024.subsystems.pivot.PivotConstants;
import frc2024.subsystems.shooter.ShootStateInterpolatingTreeMap.ShootState;

public class ShootingUtils {
  public record ShotParameters(
      Rotation2d targetHeading,
      ShootState shootState,
      double effectiveDistance,
      double actualDistance) {}

  private static final LinearFilter headingFilter = LinearFilter.movingAverage(5);

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
    if (pointedAwayFromGoal()) {
      calculated.setPivotAngle(
          Rotation2d.fromRotations(PivotGoal.HOME_INTAKE.getTarget().getAsDouble()));
    } else if (!withinRange()
        && FieldConstants.CENTER_AREA.contains(currentTranslation)) { // Feed to wing
      targetTranslation = AllianceFlipUtil.MirroredTranslation2d(new Translation2d(2.0, 6.6));
      calculated.setPivotAngle(
          Rotation2d.fromRotations(PivotGoal.FEED_TO_WING.getTarget().getAsDouble()));
    } else if (AllianceFlipUtil.get(FieldConstants.RED_WING_AREA, FieldConstants.BLUE_WING_AREA)
        .contains(currentTranslation)) { // Feed to center
      targetTranslation = AllianceFlipUtil.MirroredTranslation2d(new Translation2d(6.85, 6.65));
      calculated.setPivotAngle(
          Rotation2d.fromRotations(PivotGoal.FEED_TO_CENTER.getTarget().getAsDouble()));
    } else {
      calculated.setPivotAngle(
          Rotation2d.fromDegrees(
              MathUtil.clamp(
                  getPivotAngleToGoal(effectiveDistance, FieldConstants.SPEAKER_OPENING.getZ())
                      .getDegrees(),
                  subsystems.elevator().getMeasuredHeight().getInches() > 1.5 ? 0 : 4,
                  Units.rotationsToDegrees(PivotGoal.SUB.getTarget().getAsDouble()))));
      calculated.setElevatorHeight(mapped.elevatorHeight);
    }
    calculated.setVelocityRPM(
        (subsystems.conveyor().hasNote() && withinRange()) || DriverStation.isAutonomous()
            ? mapped.velocityRPM
            : 2000);

    Translation2d raw = targetTranslation.minus(currentTranslation);
    Rotation2d targetHeading =
        raw.minus(new Translation2d(0, shotOffset.getY()))
            .getAngle()
            .minus(new Rotation2d(Math.PI));

    return new ShotParameters(targetHeading, calculated, effectiveDistance, actualDistance);
  }

  private static Rotation2d getPivotAngleToGoal(double horizontalDistance, double goalHeight) {
    return ScreamMath.calculateAngleToPoint(
        RobotState.getPivotRootPosition().get(),
        new Translation2d(
            horizontalDistance,
            goalHeight - PivotConstants.SHOOTER_DISTANCE_FROM_AXLE.getMeters()));
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

  public static boolean validShot(double horizontalDistance) {
    return subsystems.shooter().atGoal()
        && subsystems.elevator().atGoal()
        && subsystems.pivot().atGoal()
        && subsystems
            .drivetrain()
            .getWithinAngleThreshold(
                RobotState.getActiveShotParameters().get().targetHeading(),
                Rotation2d.fromDegrees((1 / horizontalDistance) * 13.0))
        && ScreamMath.getLinearVelocity(subsystems.drivetrain().getRobotRelativeSpeeds()) < 0.5
        && withinRange();
  }

  private static Rotation2d filterAngle(Rotation2d angle) {
    return Rotation2d.fromDegrees(
        headingFilter.calculate(Math.abs(angle.getDegrees())) * Math.signum(angle.getDegrees()));
  }
}
