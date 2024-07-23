package frc2024;

import java.sql.Driver;
import java.util.ArrayList;
import java.util.List;
import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.console.SimConsoleSource;
import org.photonvision.estimation.RotTrlTransform3d;

import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrain.SwerveDriveState;
import com.team4522.lib.math.Conversions;
import com.team4522.lib.util.AllianceFlipUtil;
import com.team4522.lib.util.Length;
import com.team4522.lib.util.LimitedSizeList;
import com.team4522.lib.util.PolygonalPoseArea;
import com.team4522.lib.util.ScreamUtil;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc2024.RobotContainer.Subsystems;
import frc2024.commands.ShootSimNote;
import frc2024.constants.Constants;
import frc2024.constants.FieldConstants;
import frc2024.constants.SimConstants;
import frc2024.dashboard.ComponentVisualizer;
import frc2024.dashboard.MechanismVisualizer;
import frc2024.subsystems.conveyor.Conveyor;
import frc2024.subsystems.elevator.Elevator;
import frc2024.subsystems.elevator.ElevatorConstants;
import frc2024.subsystems.pivot.Pivot;
import frc2024.subsystems.pivot.PivotConstants;
import frc2024.subsystems.shooter.Shooter;
import frc2024.subsystems.shooter.ShooterConstants;
import frc2024.subsystems.shooter.ShootingUtils;
import frc2024.subsystems.shooter.Shooter.Goal;
import frc2024.subsystems.shooter.ShootingUtils.ShotParameters;
import frc2024.subsystems.stabilizers.StabilizerConstants;
import frc2024.subsystems.stabilizers.Stabilizers;
import frc2024.subsystems.swerve.Drivetrain;
import lombok.Getter;
import lombok.Setter;

public class RobotState {

    private Drivetrain drivetrain;
    private Elevator elevator;
    private Pivot pivot;
    private Shooter shooter;
    private Conveyor conveyor;
    private Stabilizers stabilizers;

    public enum SuperstructureGoal{
        IDLE,
        SUB,
        SUB_DEFENDED,
        AMP,
        PASSING,
        EJECT;
    }

    @Getter
    private final Supplier<Translation2d> pivotRootPosition = () -> {
        Translation3d t = ComponentVisualizer.getShooterPose(elevator.getHeight().getMeters(), new Rotation2d()).getTranslation();
        return new Translation2d(t.getX(), t.getZ());
    };

    @Getter @Setter
    private Translation3d activeSpeaker = AllianceFlipUtil.MirroredTranslation3d(FieldConstants.SPEAKER_OPENING);

    @Getter
    private final Supplier<ShotParameters> activeShotParameters =
        () -> ShootingUtils.calculateShotParameters(drivetrain.getPose().getTranslation(), getActiveSpeaker().toTranslation2d());

    @Getter
    private final Supplier<Translation3d[]> activeTrajectory = 
        () -> ShootingUtils.calculateSimpleTrajectory(
                    drivetrain.getPose(),
                    drivetrain.getPose().getTranslation().getDistance(activeSpeaker.toTranslation2d()) + 1);

    public final LimitedSizeList<Supplier<Pose3d>> activeNotes = new LimitedSizeList<>(SimConstants.MAX_SIM_NOTES);

    static{
        MechanismVisualizer.setDimensions(SimConstants.MECH_WIDTH, SimConstants.MECH_HEIGHT);
    }
    private final MechanismVisualizer elevatorVisualizer = 
        new MechanismVisualizer("Elevator")
            .withStaticAngle(Rotation2d.fromDegrees(80))
            .withDynamicLength(
                () -> elevator.getHeight().plus(ElevatorConstants.HOME_HEIGHT_FROM_FLOOR), 
                () -> Elevator.rotationsToLength(elevator.getGoal().getTargetRotations().getAsDouble(), ElevatorConstants.PULLEY_CIRCUMFERENCE.getInches()).plus(ElevatorConstants.HOME_HEIGHT_FROM_FLOOR))
            .withStaticPosition(new Translation2d(SimConstants.ELEVATOR_X, 0));

    private final MechanismVisualizer shooterFrontVisualizer = 
        new MechanismVisualizer("Pivot Front")
            .withStaticLength(Length.fromInches(17))
            .withDynamicAngle(
                () -> pivot.getAngle().unaryMinus(), 
                () -> Rotation2d.fromRotations(-pivot.getGoal().getTargetRotations().getAsDouble()))
            .withDynamicPosition(() -> pivotRootPosition.get().plus(new Translation2d(SimConstants.ELEVATOR_X, 0)));

    private final MechanismVisualizer shooterBackVisualizer = 
        new MechanismVisualizer("Pivot Back")
            .withStaticLength(Length.fromInches(9))
            .withDynamicAngle(
                () -> pivot.getAngle().unaryMinus().plus(Rotation2d.fromRotations(0.5)), 
                () -> Rotation2d.fromRotations(0.5 - pivot.getGoal().getTargetRotations().getAsDouble()))
            .withDynamicPosition(() -> pivotRootPosition.get().plus(new Translation2d(SimConstants.ELEVATOR_X, 0)));

    public RobotState(Subsystems subsystems){
        drivetrain = subsystems.drivetrain();
        elevator = subsystems.elevator();
        pivot = subsystems.pivot();
        shooter = subsystems.shooter();
        conveyor = subsystems.conveyor();
        stabilizers = subsystems.stabilizers();
    }

    public static Command shootSimNoteCommand(){
        return Commands.runOnce(() -> new ShootSimNote(RobotContainer.getSubsystems()).schedule());
    }

    public Command setSuperstructureGoalCommand(SuperstructureGoal goal){
        Supplier<Command> command;
        command = () -> {
            switch(goal){
                case AMP:
                    return elevator.moveToGoal(Elevator.Goal.AMP)
                        .alongWith(pivot.setGoalCommand(Pivot.Goal.AMP))
                        .alongWith(shooter.setGoalCommand(Shooter.Goal.TRACKING));
                case PASSING:
                    return elevator.moveToGoal(Elevator.Goal.TRACKING)
                        .alongWith(pivot.setGoalCommand(Pivot.Goal.TRACKING))
                        .alongWith(shooter.setGoalCommand(Shooter.Goal.TRACKING));
                case SUB:
                    return elevator.moveToGoal(Elevator.Goal.SUB)
                        .alongWith(pivot.setGoalCommand(Pivot.Goal.SUB))
                        .alongWith(shooter.setGoalCommand(Shooter.Goal.SUB));
                case SUB_DEFENDED:
                    return elevator.moveToGoal(Elevator.Goal.SUB_DEFENDED)
                        .alongWith(pivot.setGoalCommand(Pivot.Goal.SUB_DEFENDED))
                        .alongWith(shooter.setGoalCommand(Shooter.Goal.SUB));
                case EJECT:
                    return elevator.moveToGoal(Elevator.Goal.EJECT)
                        .alongWith(pivot.setGoalCommand(Pivot.Goal.EJECT))
                        .alongWith(shooter.setGoalCommand(Shooter.Goal.TRACKING));
                case IDLE:
                default:
                    return elevator.moveToGoal(Elevator.Goal.TRACKING)
                        .alongWith(pivot.setGoalCommand(Pivot.Goal.TRACKING))
                        .alongWith(shooter.setGoalCommand(Shooter.Goal.TRACKING));
            }};
        return command.get();
    }

    public void outputTelemetry(){
        Logger.recordOutput("Simulation/ActiveNotes", activeNotes.stream().map(Supplier::get).toArray(Pose3d[]::new));
        Logger.recordOutput("Simulation/NoteTrajectory", getActiveTrajectory().get());
        Logger.recordOutput("RobotState/HorizontalDistanceFromGoal", getActiveShotParameters().get().effectiveDistance());
        Logger.recordOutput("Simulation/ComponentPoses", 
            new Pose3d[]{
                ComponentVisualizer.getElevStage1Pose(elevator.getHeight().getMeters()), 
                ComponentVisualizer.getElevStage2Pose(elevator.getHeight().getMeters()), 
                ComponentVisualizer.getShooterPose(elevator.getHeight().getMeters(), pivot.getAngle()), 
                ComponentVisualizer.getStabilizersPose(stabilizers.getAngle())
            });
        Logger.recordOutput("shooter", shooter.atGoal());
        Logger.recordOutput("pivot", pivot.atGoal());
        Logger.recordOutput("ele", elevator.atGoal());
        Logger.recordOutput("locked", drivetrain.getWithinAngleThreshold(getActiveShotParameters().get().targetHeading(), Rotation2d.fromDegrees(3.0)));
        Logger.recordOutput("withinrange", ShootingUtils.withinRange());
        Logger.recordOutput("spped", ScreamUtil.getLinearSpeed(drivetrain.getRobotRelativeSpeeds()));
    }

    public void telemeterizeDrivetrain(SwerveDriveState state){
        Logger.recordOutput("RobotState/RobotPose", state.Pose);
        Logger.recordOutput("RobotState/Subsystems/Swerve/MeasuredStates", state.ModuleStates);
        Logger.recordOutput("RobotState/Subsystems/Swerve/SetpointStates", state.ModuleTargets);
    }
}
