package frc2024;

import java.sql.Driver;
import java.util.ArrayList;
import java.util.List;
import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.console.SimConsoleSource;
import org.photonvision.estimation.RotTrlTransform3d;

import com.SCREAMLib.data.Length;
import com.SCREAMLib.data.LimitedSizeList;
import com.SCREAMLib.util.AllianceFlipUtil;
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrain.SwerveDriveState;
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
import frc2024.dashboard.MechanismVisualizer;
import frc2024.logging.ComponentConstants;
import frc2024.subsystems.conveyor.Conveyor;
import frc2024.subsystems.elevator.Elevator;
import frc2024.subsystems.elevator.ElevatorConstants;
import frc2024.subsystems.elevator.Elevator.ElevatorGoal;
import frc2024.subsystems.pivot.Pivot;
import frc2024.subsystems.pivot.PivotConstants;
import frc2024.subsystems.pivot.Pivot.PivotGoal;
import frc2024.subsystems.shooter.Shooter;
import frc2024.subsystems.shooter.ShooterConstants;
import frc2024.subsystems.shooter.ShootingUtils;
import frc2024.subsystems.shooter.Shooter.ShooterGoal;
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
        HOME_INTAKE,
        TRAP_INTAKE,
        SUB,
        SUB_DEFENDED,
        AMP,
        PASSING,
        EJECT;
    }

    @Getter
    private final Supplier<Translation2d> pivotRootPosition = () -> {
        Translation3d t = ComponentConstants.getShooterPose(elevator.getHeight().getMeters(), new Rotation2d()).getTranslation();
        return new Translation2d(t.getX(), t.getZ());
    };

    @Getter @Setter
    private Supplier<Translation3d> activeSpeaker = () -> AllianceFlipUtil.MirroredTranslation3d(FieldConstants.SPEAKER_OPENING);

    @Getter
    private final Supplier<ShotParameters> activeShotParameters =
        () -> ShootingUtils.calculateSimpleShotParameters(drivetrain.getPose().getTranslation(), getActiveSpeaker().get().toTranslation2d());

    @Getter
    private final Supplier<Translation3d[]> activeTrajectory = 
        () -> ShootingUtils.calculateSimpleTrajectory(
                    drivetrain.getPose(),
                    drivetrain.getPose().getTranslation().getDistance(activeSpeaker.get().toTranslation2d()) + 1);

    private final LimitedSizeList<Supplier<Pose3d>> activeNotes = new LimitedSizeList<>(SimConstants.MAX_SIM_NOTES);

    static{
        MechanismVisualizer.setDimensions(SimConstants.MECH_WIDTH, SimConstants.MECH_HEIGHT);
    }
    private final MechanismVisualizer elevatorVisualizer = 
        new MechanismVisualizer("Elevator")
            .withStaticAngle(Rotation2d.fromDegrees(80))
            .withDynamicLength(
                () -> elevator.getHeight().plus(ElevatorConstants.HOME_HEIGHT_FROM_FLOOR), 
                () -> Length.fromRotations(elevator.getGoal().target().getAsDouble(), ElevatorConstants.PULLEY_CIRCUMFERENCE).plus(ElevatorConstants.HOME_HEIGHT_FROM_FLOOR))
            .withStaticPosition(new Translation2d(SimConstants.ELEVATOR_X, 0));

    private final MechanismVisualizer shooterFrontVisualizer = 
        new MechanismVisualizer("Pivot Front")
            .withStaticLength(Length.fromInches(17))
            .withDynamicAngle(
                () -> pivot.getAngle().unaryMinus(), 
                () -> Rotation2d.fromRotations(-pivot.getGoal().target().getAsDouble()))
            .withDynamicPosition(() -> pivotRootPosition.get().plus(new Translation2d(SimConstants.ELEVATOR_X, 0)));

    private final MechanismVisualizer shooterBackVisualizer = 
        new MechanismVisualizer("Pivot Back")
            .withStaticLength(Length.fromInches(9))
            .withDynamicAngle(
                () -> pivot.getAngle().unaryMinus().plus(Rotation2d.fromRotations(0.5)), 
                () -> Rotation2d.fromRotations(0.5 - pivot.getGoal().target().getAsDouble()))
            .withDynamicPosition(() -> pivotRootPosition.get().plus(new Translation2d(SimConstants.ELEVATOR_X, 0)));

    public RobotState(Subsystems subsystems){
        drivetrain = subsystems.drivetrain();
        elevator = subsystems.elevator();
        pivot = subsystems.pivot();
        shooter = subsystems.shooter();
        conveyor = subsystems.conveyor();
        stabilizers = subsystems.stabilizers();
    }

    public void addActiveNote(Supplier<Pose3d> note){
        activeNotes.add(note);
    }

    public static Command shootSimNoteCommand(){
        return Commands.runOnce(() -> new ShootSimNote(RobotContainer.getSubsystems()).schedule());
    }

    public Command setSuperstructureGoalCommand(SuperstructureGoal goal){
        Supplier<Command> command;
        command = () -> {
            switch(goal){
                case HOME_INTAKE:
                    return elevator.applyGoal(ElevatorGoal.HOME_INTAKE)
                        .alongWith(pivot.applyGoal(PivotGoal.HOME_INTAKE))
                        .alongWith(shooter.applyGoal(Shooter.ShooterGoal.TRACKING));
                case TRAP_INTAKE:
                    return elevator.applyGoal(ElevatorGoal.TRAP_INTAKE)
                        .alongWith(pivot.applyGoal(PivotGoal.TRAP_INTAKE))
                        .alongWith(shooter.applyGoal(Shooter.ShooterGoal.TRACKING));
                case AMP:
                    return elevator.applyGoal(ElevatorGoal.AMP)
                        .alongWith(pivot.applyGoal(PivotGoal.AMP))
                        .alongWith(shooter.applyGoal(Shooter.ShooterGoal.TRACKING));
                case PASSING:
                    return elevator.applyGoal(ElevatorGoal.TRACKING)
                        .alongWith(pivot.applyGoal(PivotGoal.TRACKING))
                        .alongWith(shooter.applyGoal(Shooter.ShooterGoal.TRACKING));
                case SUB:
                    return elevator.applyGoal(ElevatorGoal.SUB)
                        .alongWith(pivot.applyGoal(PivotGoal.SUB))
                        .alongWith(shooter.applyGoal(Shooter.ShooterGoal.SUB));
                case SUB_DEFENDED:
                    return elevator.applyGoal(ElevatorGoal.SUB_DEFENDED)
                        .alongWith(pivot.applyGoal(PivotGoal.SUB_DEFENDED))
                        .alongWith(shooter.applyGoal(Shooter.ShooterGoal.SUB));
                case EJECT:
                    return elevator.applyGoal(ElevatorGoal.EJECT)
                        .alongWith(pivot.applyGoal(PivotGoal.EJECT))
                        .alongWith(shooter.applyGoal(Shooter.ShooterGoal.TRACKING));
                case IDLE:
                default:
                    return elevator.applyGoal(ElevatorGoal.TRACKING)
                        .alongWith(pivot.applyGoal(PivotGoal.TRACKING))
                        .alongWith(shooter.applyGoal(Shooter.ShooterGoal.TRACKING));
            }};
        return command.get();
    }

    public void outputTelemetry(){
        Logger.recordOutput("Simulation/ActiveNotes", activeNotes.stream().map(Supplier::get).toArray(Pose3d[]::new));
        Logger.recordOutput("Simulation/NoteTrajectory", getActiveTrajectory().get());
        Logger.recordOutput("RobotState/HorizontalDistanceFromGoal", getActiveShotParameters().get().effectiveDistance());
        Logger.recordOutput("Simulation/ComponentPoses", 
            new Pose3d[]{
                ComponentConstants.getElevStage1Pose(elevator.getHeight().getMeters()), 
                ComponentConstants.getElevStage2Pose(elevator.getHeight().getMeters()), 
                ComponentConstants.getShooterPose(elevator.getHeight().getMeters(), pivot.getAngle()), 
                ComponentConstants.getStabilizersPose(stabilizers.getAngle())
            });
    }

    public void telemeterizeDrivetrain(SwerveDriveState state){
        Logger.recordOutput("RobotState/RobotPose", state.Pose);
        Logger.recordOutput("RobotState/Subsystems/Swerve/MeasuredStates", state.ModuleStates);
        Logger.recordOutput("RobotState/Subsystems/Swerve/SetpointStates", state.ModuleTargets);
    }
}
