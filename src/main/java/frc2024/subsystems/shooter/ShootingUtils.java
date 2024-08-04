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
import frc2024.constants.Constants;
import frc2024.constants.FieldConstants;
import frc2024.constants.SimConstants;
import frc2024.logging.ComponentConstants;
import frc2024.subsystems.pivot.Pivot.PivotGoal;
import frc2024.subsystems.pivot.PivotConstants;
import lombok.Getter;
import lombok.Setter;

public class ShootingUtils {
  public record ShotParameters(
      Rotation2d targetHeading, ShootState shootState, double effectiveDistance) {}

  private static final LinearFilter headingFilter = LinearFilter.movingAverage(5);

  private static final Subsystems subsystems = RobotContainer.getSubsystems();

  public static Translation3d[] calculateTrajectory(
      Rotation2d launchAngle, double initialVelocity, Pose2d pose) {
    Translation3d[] trajectoryPoints = new Translation3d[SimConstants.NUM_TRAJECTORY_POINTS + 1];

    Translation3d temp =
        ComponentConstants.getShooterPose(
                RobotContainer.getSubsystems().elevator().getHeight().getMeters(), new Rotation2d())
            .getTranslation(); // new
    // Translation2d(absoluteHeight.minus(PivotConstants.AXLE_DISTANCE_FROM_ELEVATOR_TOP).plus(PivotConstants.SHOOTER_DISTANCE_FROM_AXLE).getMeters(),
    // Rotation2d.fromDegrees(80));
    Translation2d shooterRootPos =
        new Translation2d(temp.getX(), temp.getZ())
            .plus(
                new Translation2d(
                    PivotConstants.SHOOTER_DISTANCE_FROM_AXLE.getMeters(),
                    RobotContainer.getSubsystems().pivot().getAngle()));

    launchAngle = launchAngle.plus(new Rotation2d(Math.PI)).unaryMinus();
    double totalTime =
        (initialVelocity * launchAngle.getSin()
                + Math.sqrt(
                    Math.pow(initialVelocity * launchAngle.getSin(), 2)
                        + 2 * Constants.GRAVITY * temp.getY()))
            / Constants.GRAVITY;

    double timeInterval = totalTime / SimConstants.NUM_TRAJECTORY_POINTS;

    int index = 0;

    for (int i = 0; i <= SimConstants.NUM_TRAJECTORY_POINTS; i++) {
      double time = i * timeInterval;
      double x = initialVelocity * launchAngle.getCos() * time;
      double y =
          (initialVelocity * launchAngle.getSin() * time)
              - (0.5 * Constants.GRAVITY * time * time)
              + shooterRootPos.getY();
      double z = 0;
      if (y < 0) {
        y = 0;
      }

      Translation3d rotatedPoint =
          ScreamMath.rotatePoint(
              new Translation3d(x, z, y).plus(new Translation3d(shooterRootPos.getX(), 0.0, 0.0)),
              pose.getRotation());

      double relX = rotatedPoint.getX() + pose.getX();
      double relY = rotatedPoint.getY() + pose.getY();
      double relZ = rotatedPoint.getZ();

      trajectoryPoints[index++] = new Translation3d(relX, relY, relZ);
    }

    return trajectoryPoints;
  }

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

  public static ShotParameters calculateShotParameters(
      Translation2d currentTranslation, Translation2d targetTranslation) {
    Translation2d shotOffset =
        getMoveWhileShootOffset(subsystems.drivetrain().getFieldRelativeSpeeds());
    double horizontalDistance =
        currentTranslation.getDistance(targetTranslation) + shotOffset.getX() / 4.0;
    Translation2d raw = targetTranslation.minus(currentTranslation);
    Rotation2d targetHeading =
        raw.minus(new Translation2d(0, shotOffset.getY()).div(3))
            .getAngle()
            .minus(new Rotation2d(Math.PI));
    ShootState calculated = ShooterConstants.SHOOTING_MAP.get(horizontalDistance);
    double velocity = withinRange() || DriverStation.isAutonomous() ? calculated.velocityRPM : 2000;

    Rotation2d adjustedAngle;
    if (subsystems
            .drivetrain()
            .getWithinAngleThreshold(
                AllianceFlipUtil.MirroredRotation2d(Rotation2d.fromDegrees(180)),
                Rotation2d.fromDegrees(90))
        || !withinRange()) {
      adjustedAngle =
          Rotation2d.fromRotations(PivotGoal.HOME_INTAKE.getTargetRotations().getAsDouble());
    } else {
      adjustedAngle =
          Rotation2d.fromDegrees(
              MathUtil.clamp(
                  calculated.pivotAngle.plus(PivotConstants.MAP_OFFSET).getDegrees(),
                  subsystems.elevator().getHeight().getInches() > 1.5 ? 0 : 4,
                  Units.rotationsToDegrees(PivotGoal.SUB.getTargetRotations().getAsDouble())));
    }

    ShootState adjusted =
        new ShootState(
            adjustedAngle,
            subsystems
                    .drivetrain()
                    .getWithinAngleThreshold(
                        AllianceFlipUtil.MirroredRotation2d(Rotation2d.fromDegrees(180)),
                        Rotation2d.fromDegrees(90))
                ? 0
                : calculated.elevatorHeight,
            velocity);

    return new ShotParameters(targetHeading, adjusted, horizontalDistance);
  }

  public static ShotParameters calculateSimpleShotParameters(
      Translation2d currentTranslation, Translation2d targetTranslation) {
    Translation2d shotOffset =
        getMoveWhileShootOffset(subsystems.drivetrain().getFieldRelativeSpeeds());

    double horizontalDistance =
        currentTranslation.getDistance(targetTranslation) + shotOffset.getX();

    ShootState map = ShooterConstants.SHOOTING_MAP.get(horizontalDistance);
    ShootState calculated = new ShootState();
    if (pointedAwayFromGoal()) {
      calculated.setPivotAngle(
          Rotation2d.fromRotations(PivotGoal.HOME_INTAKE.getTargetRotations().getAsDouble()));
    } else if (!withinRange()
        && FieldConstants.CENTER_AREA.isPoseWithinArea(currentTranslation)) { // Feed to wing
      targetTranslation = AllianceFlipUtil.MirroredTranslation2d(new Translation2d(2.0, 6.6));
      calculated.setPivotAngle(
          Rotation2d.fromRotations(PivotGoal.FEED_TO_WING.getTargetRotations().getAsDouble()));
    } else if (AllianceFlipUtil.PoseArea(FieldConstants.OPPOSING_WING_AREA)
        .isPoseWithinArea(currentTranslation)) { // Feed to center
      targetTranslation = AllianceFlipUtil.MirroredTranslation2d(new Translation2d(6.85, 6.65));
      calculated.setPivotAngle(
          Rotation2d.fromRotations(PivotGoal.FEED_TO_CENTER.getTargetRotations().getAsDouble()));
    } else {
      calculated.setPivotAngle(
          Rotation2d.fromDegrees(
              MathUtil.clamp(
                  getAngleToGoal(horizontalDistance, FieldConstants.SPEAKER_OPENING.getZ())
                      .getDegrees(),
                  subsystems.elevator().getHeight().getInches() > 1.5 ? 0 : 4,
                  Units.rotationsToDegrees(PivotGoal.SUB.getTargetRotations().getAsDouble()))));
      calculated.setElevatorHeight(map.elevatorHeight);
    }
    calculated.setVelocityRPM(
        (subsystems.conveyor().hasNote() && withinRange()) || DriverStation.isAutonomous()
            ? map.velocityRPM
            : 2000);

    Translation2d raw = targetTranslation.minus(currentTranslation);
    Rotation2d targetHeading =
        raw.minus(new Translation2d(0, shotOffset.getY()))
            .getAngle()
            .minus(new Rotation2d(Math.PI));

    return new ShotParameters(targetHeading, calculated, horizontalDistance);
  }

  private static Rotation2d getAngleToGoal(double horizontalDistance, double goalHeight) {
    return ScreamMath.calculateAngleToPoint(
        RobotState.getPivotRootPosition().get(),
        new Translation2d(
            horizontalDistance,
            goalHeight - PivotConstants.SHOOTER_DISTANCE_FROM_AXLE.getMeters()));
  }

  public static Translation2d getMoveWhileShootOffset(ChassisSpeeds robotSpeeds) {
    Translation2d temp = DataConversions.chassisSpeedsToTranslation(robotSpeeds);
    return new Translation2d(
        temp.getX() / 4.0, temp.getY() * AllianceFlipUtil.getDirectionCoefficient() / 3.0);
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
        && ScreamMath.getLinearSpeed(subsystems.drivetrain().getRobotRelativeSpeeds()) < 0.5
        && withinRange();
  }

  private static Rotation2d filterAngle(Rotation2d angle) {
    return Rotation2d.fromDegrees(
        headingFilter.calculate(Math.abs(angle.getDegrees())) * Math.signum(angle.getDegrees()));
  }

  public static class ShootState {

    @Getter @Setter Rotation2d pivotAngle;

    @Getter @Setter double elevatorHeight, velocityRPM;

    public ShootState(Rotation2d pivotAngle, double elevatorHeight, double velocityRPM) {
      this.pivotAngle = pivotAngle;
      this.elevatorHeight = elevatorHeight;
      this.velocityRPM = velocityRPM;
    }

    public ShootState() {
      this.pivotAngle = new Rotation2d();
      this.elevatorHeight = 0;
      this.velocityRPM = 0;
    }
  }
}
