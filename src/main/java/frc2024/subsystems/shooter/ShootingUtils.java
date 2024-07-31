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
import frc2024.constants.Constants;
import frc2024.constants.FieldConstants;
import frc2024.constants.SimConstants;
import frc2024.dashboard.ComponentConstants;
import frc2024.subsystems.elevator.ElevatorConstants;
import frc2024.subsystems.pivot.Pivot;
import frc2024.subsystems.pivot.PivotConstants;
import frc2024.subsystems.pivot.Pivot.PivotGoal;

public class ShootingUtils {

    public record ShootState(Rotation2d pivotAngle, double elevatorHeight, double velocityRPM){}
    public record ShotParameters(Rotation2d targetHeading, ShootState shootState, double effectiveDistance){}

    private static final LinearFilter headingFilter = LinearFilter.movingAverage(5);

    private static final Subsystems subsystems = RobotContainer.getSubsystems();

    public static Translation3d[] calculateTrajectory(Rotation2d launchAngle, double initialVelocity, Pose2d pose) {
        Translation3d[] trajectoryPoints = new Translation3d[SimConstants.NUM_TRAJECTORY_POINTS + 1];

        Translation3d temp = ComponentConstants.getShooterPose(RobotContainer.getSubsystems().elevator().getHeight().getMeters(), new Rotation2d()).getTranslation();//new Translation2d(absoluteHeight.minus(PivotConstants.AXLE_DISTANCE_FROM_ELEVATOR_TOP).plus(PivotConstants.SHOOTER_DISTANCE_FROM_AXLE).getMeters(), Rotation2d.fromDegrees(80));
        Translation2d shooterRootPos = new Translation2d(temp.getX(), temp.getZ()).plus(new Translation2d(PivotConstants.SHOOTER_DISTANCE_FROM_AXLE.getMeters(), RobotContainer.getSubsystems().pivot().getAngle()));

        launchAngle = launchAngle.plus(new Rotation2d(Math.PI)).unaryMinus();
        double totalTime = (initialVelocity * launchAngle.getSin() + Math.sqrt(Math.pow(initialVelocity * launchAngle.getSin(), 2) + 2 * Constants.GRAVITY * temp.getY())) / Constants.GRAVITY;

        double timeInterval = totalTime / SimConstants.NUM_TRAJECTORY_POINTS; 

        int index = 0; 

        for (int i = 0; i <= SimConstants.NUM_TRAJECTORY_POINTS; i++) {
            double time = i * timeInterval;
            double x = initialVelocity * launchAngle.getCos() * time;
            double y = (initialVelocity * launchAngle.getSin() * time) - (0.5 * Constants.GRAVITY * time * time) + shooterRootPos.getY();
            double z = 0; 
            if (y < 0) {
                y = 0; 
            }

            Translation3d rotatedPoint = ScreamMath.rotatePoint(new Translation3d(x, z, y).plus(new Translation3d(shooterRootPos.getX(), 0.0, 0.0)), pose.getRotation());

            double relX = rotatedPoint.getX() + pose.getX();
            double relY = rotatedPoint.getY() + pose.getY();
            double relZ = rotatedPoint.getZ();

            trajectoryPoints[index++] = new Translation3d(relX, relY, relZ);
        }

        return trajectoryPoints;
    }

    public static Translation3d[] calculateSimpleTrajectory(Pose2d pose, double horizontalDistance) {
        Translation2d shooterExitPos = RobotContainer.getRobotState().getPivotRootPosition().get()
            .plus(new Translation2d(PivotConstants.SHOOTER_DISTANCE_FROM_AXLE.getMeters(), subsystems.pivot().getAngle().unaryMinus().plus(Rotation2d.fromDegrees(90))))
            .plus(new Translation2d(ShooterConstants.SHOOTER_BACK_LENGTH.getMeters(), subsystems.pivot().getAngle().unaryMinus().plus(new Rotation2d(Math.PI))));;

        Translation2d endPoint2d = shooterExitPos.plus(new Translation2d(horizontalDistance, subsystems.pivot().getAngle().unaryMinus().plus(new Rotation2d(Math.PI))));

        Translation3d startPoint = ScreamMath.rotatePoint(new Translation3d(shooterExitPos.getX(), 0, shooterExitPos.getY()), pose.getRotation()).plus(new Translation3d(pose.getX(), pose.getY(), 0));
        Translation3d endPoint = ScreamMath.rotatePoint(new Translation3d(endPoint2d.getX(), 0, endPoint2d.getY()), pose.getRotation()).plus(new Translation3d(pose.getX(), pose.getY(), 0));

        return new Translation3d[]{
            startPoint, endPoint
        };
    }

    public static ShotParameters calculateShotParameters(Translation2d currentTranslation, Translation2d targetTranslation){
        ChassisSpeeds robotSpeeds = subsystems.drivetrain().getFieldRelativeSpeeds();
        double horizontalDistance = currentTranslation.getDistance(targetTranslation) + (Math.max(robotSpeeds.vxMetersPerSecond, 0) / 4.0);
        Translation2d raw = targetTranslation.minus(currentTranslation);
        Rotation2d targetHeading = raw.minus(DataConversions.chassisSpeedsToTranslation(new ChassisSpeeds(0, robotSpeeds.vyMetersPerSecond, 0)).div(3)).getAngle().minus(new Rotation2d(Math.PI));
        ShootState calculated = ShooterConstants.SHOOTING_MAP.get(horizontalDistance);
        double velocity = withinRange() || DriverStation.isAutonomous() ? calculated.velocityRPM : 2000;
        
        Rotation2d adjustedAngle;
        if(subsystems.drivetrain().getWithinAngleThreshold(AllianceFlipUtil.MirroredRotation2d(Rotation2d.fromDegrees(180)), Rotation2d.fromDegrees(90)) || !withinRange()){
            adjustedAngle = Rotation2d.fromRotations(PivotGoal.HOME_INTAKE.getTargetRotations().getAsDouble());
        } else {
            adjustedAngle = 
                Rotation2d.fromDegrees(
                    MathUtil.clamp(
                        calculated.pivotAngle.plus(PivotConstants.MAP_OFFSET).getDegrees(),
                        subsystems.elevator().getHeight().getInches() > 1.5 ? 0 : 4, 
                        Units.rotationsToDegrees(PivotGoal.SUB.getTargetRotations().getAsDouble())));
        }
        
        ShootState adjusted = new ShootState(adjustedAngle, subsystems.drivetrain().getWithinAngleThreshold(AllianceFlipUtil.MirroredRotation2d(Rotation2d.fromDegrees(180)), Rotation2d.fromDegrees(90)) ? 0 : calculated.elevatorHeight, velocity);

        return new ShotParameters(targetHeading, adjusted, horizontalDistance);
    }

    public static ShotParameters calculateSimpleShotParameters(Translation2d currentTranslation, Translation2d targetTranslation){
        ChassisSpeeds robotSpeeds = subsystems.drivetrain().getFieldRelativeSpeeds();
        double horizontalDistance = currentTranslation.getDistance(targetTranslation) + (robotSpeeds.vxMetersPerSecond / 4.0);
        Translation2d raw = targetTranslation.minus(currentTranslation);
        Rotation2d targetHeading = raw.minus(DataConversions.chassisSpeedsToTranslation(new ChassisSpeeds(0, robotSpeeds.vyMetersPerSecond, 0)).div(3)).getAngle().minus(new Rotation2d(Math.PI));
        ShootState calculated = ShooterConstants.SHOOTING_MAP.get(horizontalDistance);
        calculated = new ShootState(ScreamMath.calculateAngleToPoint(RobotContainer.getRobotState().getPivotRootPosition().get(), new Translation2d(horizontalDistance, FieldConstants.SPEAKER_OPENING.getZ() - PivotConstants.SHOOTER_DISTANCE_FROM_AXLE.getMeters())), calculated.elevatorHeight, calculated.velocityRPM);
        double velocity = withinRange() || DriverStation.isAutonomous() ? calculated.velocityRPM : 2000;
        
        Rotation2d adjustedAngle;
        if(subsystems.drivetrain().getWithinAngleThreshold(AllianceFlipUtil.MirroredRotation2d(Rotation2d.fromDegrees(180)), Rotation2d.fromDegrees(90)) || !withinRange()){
            adjustedAngle = Rotation2d.fromRotations(PivotGoal.HOME_INTAKE.getTargetRotations().getAsDouble());
        } else {
            adjustedAngle = 
                Rotation2d.fromDegrees(
                    MathUtil.clamp(
                        calculated.pivotAngle.plus(PivotConstants.MAP_OFFSET).getDegrees(),
                        subsystems.elevator().getHeight().getInches() > 1.5 ? 0 : 4, 
                        Units.rotationsToDegrees(PivotGoal.SUB.getTargetRotations().getAsDouble())));
        }
        
        ShootState adjusted = new ShootState(adjustedAngle, subsystems.drivetrain().getWithinAngleThreshold(AllianceFlipUtil.MirroredRotation2d(Rotation2d.fromDegrees(180)), Rotation2d.fromDegrees(90)) ? 0 : calculated.elevatorHeight, velocity);

        return new ShotParameters(targetHeading, adjusted, horizontalDistance);
    }

    public static boolean withinRange(){
        double horizontalDistance = RobotContainer.getSubsystems().drivetrain().getPose().getTranslation().getDistance(RobotContainer.getRobotState().getActiveSpeaker().toTranslation2d());
        return horizontalDistance < 8;
    }

    public static boolean validShot(double horizontalDistance){
        return subsystems.shooter().atGoal() && subsystems.elevator().atGoal() && subsystems.pivot().atGoal() && subsystems.drivetrain().getWithinAngleThreshold(RobotContainer.getRobotState().getActiveShotParameters().get().targetHeading(), Rotation2d.fromDegrees((1 / horizontalDistance) * 13.0)) && ScreamMath.getLinearSpeed(subsystems.drivetrain().getRobotRelativeSpeeds()) < 0.5 && withinRange();
    }

    private static Rotation2d filterAngle(Rotation2d angle){
        return Rotation2d.fromDegrees(headingFilter.calculate(Math.abs(angle.getDegrees())) * Math.signum(angle.getDegrees()));
    }
}
