package frc2024;

import java.util.HashMap;
import java.util.List;

import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrain.SwerveDriveState;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.PathPlannerTrajectory;
import com.pathplanner.lib.pathfinding.LocalADStar;
import com.pathplanner.lib.pathfinding.Pathfinder;
import com.pathplanner.lib.pathfinding.Pathfinding;
import com.team4522.lib.math.Conversions;
import com.team4522.lib.util.Length;
import com.team4522.lib.util.ScreamUtil;
import com.team4522.lib.util.ShootStateInterpolatingTreeMap;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.Field2d;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc2024.RobotContainer.Subsystems;
import frc2024.constants.Constants;
import frc2024.constants.ElevatorConstants;
import frc2024.constants.FieldConstants;
import frc2024.constants.PivotConstants;
import frc2024.constants.ShooterConstants;
import frc2024.subsystems.Elevator;
import frc2024.subsystems.Pivot;
import frc2024.subsystems.Shooter;
import frc2024.subsystems.swerve.Drivetrain;
import lombok.Getter;
import lombok.Setter;

public class RobotState {

    private Drivetrain drivetrain;
    private Elevator elevator;
    private Pivot pivot;
    private Shooter shooter;

    public record ShootState(Rotation2d pivotAngle, double elevatorHeight, double velocityRPM){}
    public record ShotParameters(Rotation2d targetHeading, ShootState shootState, double effectiveDistance){}

    private final LinearFilter angleFilter = LinearFilter.movingAverage(7);

    private final double mechWidth = Units.inchesToMeters(40);
    private final double mechHeight = Units.inchesToMeters(50);

    private final Mechanism2d superstructure = new Mechanism2d(mechWidth, mechHeight);
    private final MechanismLigament2d elevatorMech = superstructure.getRoot("Elevator", mechWidth / 2.0, 0).append(new MechanismLigament2d("Height", ElevatorConstants.HOME_HEIGHT_FROM_FLOOR.getMeters(), 80, 6, new Color8Bit(Color.kOrange)));
    private final MechanismRoot2d pivotRoot = superstructure.getRoot("Pivot", 0, 0);
    private final MechanismLigament2d pivotFrontMech = pivotRoot.append(new MechanismLigament2d("Pivot1", Units.inchesToMeters(17), 180, 10, new Color8Bit(Color.kBlue)));
    private final MechanismLigament2d pivotBackMech = pivotRoot.append(new MechanismLigament2d("Pivot2", Units.inchesToMeters(9), 0, 10, new Color8Bit(Color.kBlue)));

    private Pose2d currentPose = new Pose2d();

    public RobotState(Subsystems subsystems){
        drivetrain = subsystems.drivetrain();
        elevator = subsystems.elevator();
        pivot = subsystems.pivot();
        shooter = subsystems.shooter();
    }

    public ShotParameters getShotParameters(){
        double horizontalDistance = drivetrain.getState().Pose.getTranslation().getDistance(FieldConstants.SPEAKER_OPENING.toTranslation2d());
        Rotation2d targetHeading = FieldConstants.SPEAKER_OPENING.toTranslation2d().minus(drivetrain.getPose().getTranslation()).getAngle().plus(Rotation2d.fromDegrees(180));
        ShootState calculated = ShooterConstants.SHOOTING_MAP.get(horizontalDistance);
        double velocity = horizontalDistance > Units.feetToMeters(25) ? 2000 : calculated.velocityRPM;
        Rotation2d adjustedAngle = 
            Rotation2d.fromDegrees(
                MathUtil.clamp(
                    calculated.pivotAngle.plus(PivotConstants.MAP_OFFSET).unaryMinus().plus(PivotConstants.RELATIVE_ENCODER_TO_HORIZONTAL).getDegrees(),
                    elevator.getHeight().getInches() > 1.5 ? Units.rotationsToDegrees(Pivot.Goal.SUB.getTargetRotations().getAsDouble()) : 1, 
                    28));
        ShootState adjusted = new ShootState(adjustedAngle, calculated.elevatorHeight, velocity);

        System.out.println(horizontalDistance);

        return new ShotParameters(filterAngle(targetHeading), adjusted, horizontalDistance);
    }

    public Rotation2d filterAngle(Rotation2d angle){
        return Rotation2d.fromDegrees(angleFilter.calculate(Math.abs(angle.getDegrees())) * Math.signum(angle.getDegrees()));
    }

    public void outputTelemetry(){
        Length absElevHeight = elevator.getHeight().plus(ElevatorConstants.HOME_HEIGHT_FROM_FLOOR);
        Translation2d pivotRootPos = new Translation2d(absElevHeight.minus(PivotConstants.AXLE_DISTANCE_FROM_ELEVATOR_TOP).getMeters(), Rotation2d.fromDegrees(80));
        elevatorMech.setLength(absElevHeight.getMeters());
        pivotRoot.setPosition(pivotRootPos.getX() + mechWidth / 2.0, pivotRootPos.getY());
        pivotFrontMech.setAngle(Rotation2d.fromRotations(pivot.getPosition()).minus(PivotConstants.RELATIVE_ENCODER_TO_HORIZONTAL));
        pivotBackMech.setAngle(Rotation2d.fromRotations(0.5 + pivot.getPosition()).minus(PivotConstants.RELATIVE_ENCODER_TO_HORIZONTAL));

        SmartDashboard.putData("Superstructure", superstructure);
        SmartDashboard.putNumber("Elevator Setpoint", elevator.getSetpoint());
        SmartDashboard.putNumberArray("Pose", ScreamUtil.pose2dToArray(currentPose));
    }

    public void telemeterizeDrivetrain(SwerveDriveState state){
        currentPose = state.Pose;
    }
}
