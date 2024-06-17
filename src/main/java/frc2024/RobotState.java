package frc2024;

import java.util.function.Supplier;

import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrain.SwerveDriveState;
import com.team4522.lib.math.Conversions;
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
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import frc2024.RobotContainer.Subsystems;
import frc2024.commands.ShootSimNote;
import frc2024.constants.Constants;
import frc2024.constants.ElevatorConstants;
import frc2024.constants.FieldConstants;
import frc2024.constants.PivotConstants;
import frc2024.constants.ShooterConstants;
import frc2024.subsystems.Drivetrain;
import frc2024.subsystems.Elevator;
import frc2024.subsystems.Pivot;
import frc2024.subsystems.Shooter;
import lombok.Getter;

public class RobotState {

    private Drivetrain drivetrain;
    private Elevator elevator;
    private Pivot pivot;
    private Shooter shooter;

    public record ShootState(Rotation2d pivotAngle, double elevatorHeight, double velocityRPM){}
    public record ShotParameters(Rotation2d targetHeading, ShootState shootState, double effectiveDistance){}

    private final LinearFilter angleFilter = LinearFilter.movingAverage(5);

    private final double mechWidth = Units.inchesToMeters(40);
    private final double mechHeight = Units.inchesToMeters(50);

    private final Mechanism2d superstructure = new Mechanism2d(mechWidth, mechHeight);
    private final MechanismLigament2d elevatorMech = superstructure.getRoot("Elevator", mechWidth / 2.0, 0).append(new MechanismLigament2d("Height", ElevatorConstants.HOME_HEIGHT_FROM_FLOOR.getMeters(), 80, 6, new Color8Bit(Color.kOrange)));
    private final MechanismRoot2d pivotRoot = superstructure.getRoot("Pivot", 0, 0);
    private final MechanismLigament2d pivotFrontMech = pivotRoot.append(new MechanismLigament2d("Pivot1", Units.inchesToMeters(17), 180, 10, new Color8Bit(Color.kBlue)));
    private final MechanismLigament2d pivotBackMech = pivotRoot.append(new MechanismLigament2d("Pivot2", Units.inchesToMeters(9), 0, 10, new Color8Bit(Color.kBlue)));

    @Getter
    private final Supplier<Translation3d> activeSpeaker = () -> AllianceFlipUtil.MirroredTranslation3d(FieldConstants.SPEAKER_OPENING);

    @Getter
    private final Supplier<Translation3d[]> activeTrajectory = 
        () -> calculateTrajectory(
                    Rotation2d.fromRotations(pivot.getPosition()).minus(PivotConstants.ENCODER_TO_HORIZONTAL), 
                    Conversions.falconRPSToMechanismMPS(shooter.getVelocity(), ShooterConstants.WHEEL_CIRCUMFERENCE.getMeters(), 1.0) * 0.8, 
                    drivetrain.getPose());

    @Getter
    private final Supplier<ShotParameters> activeShotParameters =
        () -> calculateShotParameters(drivetrain.getPose().getTranslation(), getActiveSpeaker().get().toTranslation2d());

    private static final ShootSimNote shootSimNote = new ShootSimNote(RobotContainer.getSubsystems());

    public RobotState(Subsystems subsystems){
        drivetrain = subsystems.drivetrain();
        elevator = subsystems.elevator();
        pivot = subsystems.pivot();
        shooter = subsystems.shooter();
    }

    public static void shootSimNote(){
        shootSimNote.restart();
    }

    private ShotParameters calculateShotParameters(Translation2d currentTranslation, Translation2d targetTranslation){
        double horizontalDistance = currentTranslation.getDistance(targetTranslation);
        Rotation2d targetHeading = targetTranslation.minus(currentTranslation).getAngle().plus(Rotation2d.fromDegrees(180));
        ShootState calculated = ShooterConstants.SHOOTING_MAP.get(horizontalDistance);
        double velocity = horizontalDistance > Units.feetToMeters(25) ? 2000 : calculated.velocityRPM;
        Rotation2d adjustedAngle = 
            Rotation2d.fromDegrees(
                MathUtil.clamp(
                    calculated.pivotAngle.plus(PivotConstants.MAP_OFFSET).unaryMinus().plus(PivotConstants.ENCODER_TO_HORIZONTAL).getDegrees(),
                    elevator.getHeight().getInches() > 1.5 ? Units.rotationsToDegrees(Pivot.Goal.SUB.getTargetRotations().getAsDouble()) : 1, 
                    28));
        ShootState adjusted = new ShootState(adjustedAngle, calculated.elevatorHeight, velocity);

        return new ShotParameters(filterAngle(targetHeading), adjusted, horizontalDistance);
    }

    private static Translation3d[] calculateTrajectory(Rotation2d launchAngle, double initialVelocity, Pose2d pose) {
        Translation3d[] trajectoryPoints = new Translation3d[ShooterConstants.NUM_TRAJECTORY_POINTS + 1];

        Length absoluteHeight = RobotContainer.getSubsystems().elevator().getHeight().plus(ElevatorConstants.HOME_HEIGHT_FROM_FLOOR);
        Translation2d pivotRootPos = new Translation2d(absoluteHeight.minus(PivotConstants.AXLE_DISTANCE_FROM_ELEVATOR_TOP).getMeters(), Rotation2d.fromDegrees(80));

        launchAngle = launchAngle.plus(new Rotation2d(Math.PI));
        double totalTime = (initialVelocity * launchAngle.getSin() + Math.sqrt(Math.pow(initialVelocity * launchAngle.getSin(), 2) + 2 * Constants.GRAVITY * pivotRootPos.getY())) / Constants.GRAVITY;

        double timeInterval = totalTime / ShooterConstants.NUM_TRAJECTORY_POINTS; 

        int index = 0; 

        for (int i = 0; i <= ShooterConstants.NUM_TRAJECTORY_POINTS; i++) {
            double time = i * timeInterval;
            double x = initialVelocity * launchAngle.getCos() * time;
            double y = (initialVelocity * launchAngle.getSin() * time) - (0.5 * Constants.GRAVITY * time * time) + pivotRootPos.getY();
            double z = 0; 
            if (y < 0) {
                y = 0; 
            }

            Translation3d rotatedPoint = ScreamUtil.rotatePoint(new Translation3d(x, z, y).plus(new Translation3d(pivotRootPos.getX(), 0.0, 0.0)), pose.getRotation());

            double relX = rotatedPoint.getX() + pose.getX();
            double relY = rotatedPoint.getY() + pose.getY();
            double relZ = rotatedPoint.getZ();

            trajectoryPoints[index++] = new Translation3d(relX, relY, relZ);
        }

        return trajectoryPoints;
    }

    public void outputTelemetry(){
        Length absElevHeight = elevator.getHeight().plus(ElevatorConstants.HOME_HEIGHT_FROM_FLOOR);
        Translation2d pivotRootPos = new Translation2d(absElevHeight.minus(PivotConstants.AXLE_DISTANCE_FROM_ELEVATOR_TOP).getMeters(), Rotation2d.fromDegrees(80));
        elevatorMech.setLength(absElevHeight.getMeters());
        pivotRoot.setPosition(pivotRootPos.getX() + mechWidth / 2.0, pivotRootPos.getY());
        pivotFrontMech.setAngle(Rotation2d.fromRotations(pivot.getPosition()).minus(PivotConstants.ENCODER_TO_HORIZONTAL));
        pivotBackMech.setAngle(Rotation2d.fromRotations(0.5 + pivot.getPosition()).minus(PivotConstants.ENCODER_TO_HORIZONTAL));

        SmartDashboard.putData("Superstructure", superstructure);
        SmartDashboard.putNumberArray("Note Trajectory", ScreamUtil.translation3dArrayToNumArray(getActiveTrajectory().get()));
    }

    public void telemeterizeDrivetrain(SwerveDriveState state){
        SmartDashboard.putNumberArray("Pose", ScreamUtil.pose2dToArray(state.Pose));
    }

    public Rotation2d filterAngle(Rotation2d angle){
        return Rotation2d.fromDegrees(angleFilter.calculate(Math.abs(angle.getDegrees())) * Math.signum(angle.getDegrees()));
    }
}
