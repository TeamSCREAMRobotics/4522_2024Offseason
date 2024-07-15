package frc2024.subsystems.shooter;

import com.team4522.lib.util.AllianceFlipUtil;
import com.team4522.lib.util.Length;
import com.team4522.lib.util.ScreamUtil;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import frc2024.RobotContainer;
import frc2024.RobotContainer.Subsystems;
import frc2024.constants.Constants;
import frc2024.dashboard.ComponentVisualizer;
import frc2024.subsystems.elevator.ElevatorConstants;
import frc2024.subsystems.pivot.Pivot;
import frc2024.subsystems.pivot.PivotConstants;

public class ShootingUtils {

    public record ShootState(Rotation2d pivotAngle, double elevatorHeight, double velocityRPM){}
    public record ShotParameters(Rotation2d targetHeading, ShootState shootState, double effectiveDistance){}

    private static final LinearFilter headingFilter = LinearFilter.movingAverage(5);

    private static final Subsystems subsystems = RobotContainer.getSubsystems();

    public static Translation3d[] calculateTrajectory(Rotation2d launchAngle, double initialVelocity, Pose2d pose) {
        Translation3d[] trajectoryPoints = new Translation3d[ShooterConstants.NUM_TRAJECTORY_POINTS + 1];

        Translation3d temp = ComponentVisualizer.getShooterPose(RobotContainer.getSubsystems().elevator().getHeight().getMeters(), new Rotation2d()).getTranslation();//new Translation2d(absoluteHeight.minus(PivotConstants.AXLE_DISTANCE_FROM_ELEVATOR_TOP).plus(PivotConstants.SHOOTER_DISTANCE_FROM_AXLE).getMeters(), Rotation2d.fromDegrees(80));
        Translation2d shooterRootPos = new Translation2d(temp.getX(), temp.getZ()).plus(new Translation2d(PivotConstants.SHOOTER_DISTANCE_FROM_AXLE.getMeters(), RobotContainer.getSubsystems().pivot().getAngle()));


        launchAngle = launchAngle.plus(new Rotation2d(Math.PI)).unaryMinus();
        double totalTime = (initialVelocity * launchAngle.getSin() + Math.sqrt(Math.pow(initialVelocity * launchAngle.getSin(), 2) + 2 * Constants.GRAVITY * temp.getY())) / Constants.GRAVITY;

        double timeInterval = totalTime / ShooterConstants.NUM_TRAJECTORY_POINTS; 

        int index = 0; 

        for (int i = 0; i <= ShooterConstants.NUM_TRAJECTORY_POINTS; i++) {
            double time = i * timeInterval;
            double x = initialVelocity * launchAngle.getCos() * time;
            double y = (initialVelocity * launchAngle.getSin() * time) - (0.5 * Constants.GRAVITY * time * time) + shooterRootPos.getY();
            double z = 0; 
            if (y < 0) {
                y = 0; 
            }

            Translation3d rotatedPoint = ScreamUtil.rotatePoint(new Translation3d(x, z, y).plus(new Translation3d(shooterRootPos.getX(), 0.0, 0.0)), pose.getRotation());

            double relX = rotatedPoint.getX() + pose.getX();
            double relY = rotatedPoint.getY() + pose.getY();
            double relZ = rotatedPoint.getZ();

            trajectoryPoints[index++] = new Translation3d(relX, relY, relZ);
        }

        return trajectoryPoints;
    }

    public static ShotParameters calculateShotParameters(Translation2d currentTranslation, Translation2d targetTranslation){
        double horizontalDistance = currentTranslation.getDistance(targetTranslation);
        Rotation2d targetHeading = targetTranslation.minus(currentTranslation).getAngle().plus(Rotation2d.fromDegrees(180));
        ShootState calculated = ShooterConstants.SHOOTING_MAP.get(horizontalDistance);
        double velocity = withinRange() || DriverStation.isAutonomous() ? calculated.velocityRPM : 2000;
        
        Rotation2d adjustedAngle;
        if(subsystems.drivetrain().getWithinAngleThreshold(AllianceFlipUtil.MirroredRotation2d(Rotation2d.fromDegrees(180)), Rotation2d.fromDegrees(90))){
            adjustedAngle = Rotation2d.fromRotations(Pivot.Goal.HOME_INTAKE.getTargetRotations().getAsDouble());
        } else {
            adjustedAngle = 
                Rotation2d.fromDegrees(
                    MathUtil.clamp(
                        calculated.pivotAngle.plus(PivotConstants.MAP_OFFSET).getDegrees(),
                        subsystems.elevator().getHeight().getInches() > 1.5 ? 0 : 4, 
                        Units.rotationsToDegrees(Pivot.Goal.SUB.getTargetRotations().getAsDouble())));
        }
        
        ShootState adjusted = new ShootState(adjustedAngle, subsystems.drivetrain().getWithinAngleThreshold(AllianceFlipUtil.MirroredRotation2d(Rotation2d.fromDegrees(180)), Rotation2d.fromDegrees(90)) ? 0 : calculated.elevatorHeight, velocity);

        return new ShotParameters(filterAngle(targetHeading), adjusted, horizontalDistance);
    }

    public static boolean withinRange(){
        double horizontalDistance = RobotContainer.getSubsystems().drivetrain().getPose().getTranslation().getDistance(RobotContainer.getRobotState().getActiveSpeaker().toTranslation2d());
        return horizontalDistance < Units.feetToMeters(25);
    }

    public static boolean validShot(){
        return subsystems.shooter().atGoal() && subsystems.elevator().atGoal() && subsystems.pivot().atGoal() && subsystems.drivetrain().getWithinAngleThreshold(RobotContainer.getRobotState().getActiveShotParameters().get().targetHeading(), Rotation2d.fromDegrees(1.0)) && ScreamUtil.getLinearSpeed(subsystems.drivetrain().getRobotRelativeSpeeds()) < 0.5 && withinRange();
    }

    private static Rotation2d filterAngle(Rotation2d angle){
        return Rotation2d.fromDegrees(headingFilter.calculate(Math.abs(angle.getDegrees())) * Math.signum(angle.getDegrees()));
    }
}
