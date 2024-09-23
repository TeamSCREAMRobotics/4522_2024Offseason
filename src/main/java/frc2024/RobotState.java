package frc2024;

import com.SCREAMLib.data.DataConversions;
import com.SCREAMLib.data.Length;
import com.SCREAMLib.data.LimitedSizeList;
import com.SCREAMLib.util.AllianceFlipUtil;
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrain.SwerveDriveState;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc2024.RobotContainer.Subsystems;
import frc2024.commands.ShootSimNote;
import frc2024.constants.FieldConstants;
import frc2024.constants.SimConstants;
import frc2024.controlboard.Controlboard;
import frc2024.dashboard.Mechanism;
import frc2024.dashboard.MechanismVisualizer;
import frc2024.logging.ComponentConstants;
import frc2024.logging.Logger;
import frc2024.logging.NoteVisualizer;
import frc2024.subsystems.conveyor.Conveyor;
import frc2024.subsystems.conveyor.Conveyor.ConveyorGoal;
import frc2024.subsystems.elevator.Elevator;
import frc2024.subsystems.elevator.Elevator.ElevatorGoal;
import frc2024.subsystems.elevator.ElevatorConstants;
import frc2024.subsystems.pivot.Pivot;
import frc2024.subsystems.pivot.Pivot.PivotGoal;
import frc2024.subsystems.shooter.Shooter;
import frc2024.subsystems.shooter.Shooter.ShooterGoal;
import frc2024.subsystems.shooter.ShootingHelper;
import frc2024.subsystems.shooter.ShootingHelper.ShotParameters;
import frc2024.subsystems.stabilizer.Stabilizer;
import frc2024.subsystems.swerve.Drivetrain;
import frc2024.subsystems.vision.Vision;
import java.util.Set;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
import lombok.Getter;

public class RobotState {

  private static Drivetrain drivetrain;
  private static Elevator elevator;
  private static Pivot pivot;
  private static Shooter shooter;
  private static Conveyor conveyor;
  private static Stabilizer stabilizer;
  private static Vision vision;

  public enum SuperstructureGoal {
    IDLE,
    HOME_INTAKE,
    TRAP_INTAKE,
    SUB,
    SUB_DEFENDED,
    AMP,
    PASSING,
    TRACKING,
    EJECT;
  }

  public static SuperstructureGoal activeSuperstructureGoal = SuperstructureGoal.IDLE;

  @Getter
  private static final Supplier<Translation2d> pivotRootPosition =
      () ->
          DataConversions.projectTo2d(
              ComponentConstants.getShooterPose(
                      elevator.getMeasuredHeight().getMeters(), pivot.getAngle())
                  .getTranslation());

  @Getter
  private static final Supplier<Translation3d> activeSpeaker =
      () -> AllianceFlipUtil.MirroredTranslation3d(FieldConstants.ScoringLocations.SPEAKER_OPENING);

  @Getter
  private static final Supplier<ShotParameters> activeShotParameters =
      () ->
          ShootingHelper.calculateSimpleShotParameters(
              drivetrain.getPose().getTranslation(), getActiveSpeaker().get().toTranslation2d());

  @Getter
  private static final DoubleSupplier speedLimit =
      () -> {
        if (ShootingHelper.withinRange() && Controlboard.driveController.getHID().getLeftBumper()) {
          return MathUtil.clamp(activeShotParameters.get().actualDistance() / 7.0, 0.5, 1);
        } else if (Controlboard.driveController.getLeftTriggerAxis()
            > Controlboard.TRIGGER_DEADBAND) {
          return 0.5;
        } else {
          return 1;
        }
      };

  @Getter private static Supplier<Translation3d[]> activeTrajectory;

  private static final LimitedSizeList<Supplier<Pose3d>> activeNotes =
      new LimitedSizeList<>(SimConstants.MAX_SIM_NOTES);

  private final Mechanism elevatorMech =
      new Mechanism("Elevator")
          .withStaticAngle(Rotation2d.fromDegrees(80))
          .withDynamicLength(
              () -> elevator.getMeasuredHeight().plus(ElevatorConstants.HOME_HEIGHT_FROM_FLOOR),
              () ->
                  Length.fromRotations(
                          elevator.getGoal().target().getAsDouble(),
                          ElevatorConstants.PULLEY_CIRCUMFERENCE)
                      .plus(ElevatorConstants.HOME_HEIGHT_FROM_FLOOR))
          .withStaticPosition(new Translation2d(SimConstants.MECH_ELEVATOR_X, 0));

  private final Mechanism shooterFrontMech =
      new Mechanism("Pivot Front")
          .withStaticLength(Length.fromInches(17))
          .withDynamicAngle(
              () -> pivot.getAngle().unaryMinus(),
              () -> Rotation2d.fromRotations(-pivot.getGoal().target().getAsDouble()))
          .withDynamicPosition(
              () ->
                  pivotRootPosition.get().plus(new Translation2d(SimConstants.MECH_ELEVATOR_X, 0)));

  private final Mechanism shooterBackMech =
      new Mechanism("Pivot Back")
          .withStaticLength(Length.fromInches(9))
          .withDynamicAngle(
              () -> pivot.getAngle().unaryMinus().plus(Rotation2d.fromRotations(0.5)),
              () -> Rotation2d.fromRotations(0.5 - pivot.getGoal().target().getAsDouble()))
          .withDynamicPosition(
              () ->
                  pivotRootPosition.get().plus(new Translation2d(SimConstants.MECH_ELEVATOR_X, 0)));

  private final MechanismVisualizer mechanismVisualizer =
      new MechanismVisualizer(
          RobotState::telemeterizeMechanisms, elevatorMech, shooterFrontMech, shooterBackMech);

  public RobotState(Subsystems subsystems) {
    drivetrain = subsystems.drivetrain();
    elevator = subsystems.elevator();
    pivot = subsystems.pivot();
    shooter = subsystems.shooter();
    conveyor = subsystems.conveyor();
    stabilizer = subsystems.stabilizer();
    vision = subsystems.vision();

    mechanismVisualizer.setEnabled(true);

    if (Robot.isSimulation()) {
      activeTrajectory =
          () ->
              ShootingHelper.calculateSimpleTrajectory(
                  drivetrain.getPose(),
                  drivetrain
                      .getPose()
                      .getTranslation()
                      .getDistance(activeSpeaker.get().toTranslation2d()));
    }
  }

  public static void addActiveNote(Supplier<Pose3d> note) {
    activeNotes.add(note);
  }

  public static Command shootSimNoteCommand() {
    return Commands.runOnce(() -> new ShootSimNote(RobotContainer.getSubsystems()).schedule());
  }

  public Command applySuperstructureGoal(SuperstructureGoal goal) {
    return Commands.defer(
        () -> {
          activeSuperstructureGoal = goal;
          switch (goal) {
            case HOME_INTAKE:
              return Commands.parallel(
                  elevator.applyGoal(ElevatorGoal.HOME_INTAKE),
                  pivot.applyGoal(PivotGoal.HOME_INTAKE),
                  shooter.applyGoal(ShooterGoal.TRACKING));
            case TRAP_INTAKE:
              return Commands.parallel(
                  elevator.applyGoal(ElevatorGoal.TRAP_INTAKE),
                  pivot.applyGoal(PivotGoal.TRAP_INTAKE),
                  shooter.applyGoal(ShooterGoal.TRACKING));
            case AMP:
              return Commands.parallel(
                  elevator.applyGoal(ElevatorGoal.AMP),
                  pivot.applyGoal(PivotGoal.AMP),
                  shooter.applyGoal(ShooterGoal.TRACKING));
            case PASSING:
              return Commands.parallel(
                  elevator.applyGoal(ElevatorGoal.TRACKING),
                  pivot.applyGoal(PivotGoal.TRACKING),
                  shooter.applyGoal(ShooterGoal.TRACKING));
            case SUB:
              return Commands.parallel(
                  elevator.applyGoal(ElevatorGoal.SUB),
                  pivot.applyGoal(PivotGoal.SUB),
                  shooter.applyGoal(ShooterGoal.SUB));
            case SUB_DEFENDED:
              return Commands.parallel(
                  elevator.applyGoal(ElevatorGoal.SUB_DEFENDED),
                  pivot.applyGoal(PivotGoal.SUB_DEFENDED),
                  shooter.applyGoal(ShooterGoal.SUB));
            case EJECT:
              return Commands.parallel(
                  elevator.applyGoal(ElevatorGoal.EJECT),
                  pivot.applyGoal(PivotGoal.EJECT),
                  shooter.applyGoal(ShooterGoal.TRACKING));
            case TRACKING:
              return Commands.parallel(
                  elevator.applyGoal(ElevatorGoal.TRACKING),
                  pivot.applyGoal(PivotGoal.TRACKING),
                  shooter.applyGoal(ShooterGoal.TRACKING));
            case IDLE:
            default:
              return Commands.parallel(
                  elevator.applyGoal(ElevatorGoal.TRACKING),
                  pivot.applyGoal(PivotGoal.TRACKING),
                  shooter.applyGoal(ShooterGoal.TRACKING));
          }
        },
        Set.of(elevator, pivot, shooter));
  }

  public static void outputTelemetry() {
    if (Robot.isSimulation()) {
      Logger.log(
          "Simulation/ActiveNotes", activeNotes.stream().map(Supplier::get).toArray(Pose3d[]::new));
      Logger.log("Simulation/NoteTrajectory", getActiveTrajectory().get());
    }
    Logger.log(
        "RobotState/HorizontalDistanceFromGoal",
        getActiveShotParameters().get().effectiveDistance());
    Logger.log(
        "Simulation/MeasuredComponentPoses",
        new Pose3d[] {
          ComponentConstants.getElevStage1Pose(elevator.getMeasuredHeight().getMeters()),
          ComponentConstants.getElevStage2Pose(elevator.getMeasuredHeight().getMeters()),
          ComponentConstants.getShooterPose(
              elevator.getMeasuredHeight().getMeters(), pivot.getAngle()),
          ComponentConstants.getStabilizerPose(stabilizer.getAngle())
        });
    Logger.log(
        "Simulation/SetpointComponentPoses",
        new Pose3d[] {
          ComponentConstants.getElevStage1Pose(elevator.getSetpointHeight().getMeters()),
          ComponentConstants.getElevStage2Pose(elevator.getSetpointHeight().getMeters()),
          ComponentConstants.getShooterPose(
              elevator.getSetpointHeight().getMeters(),
              Rotation2d.fromRotations(pivot.getSetpoint())),
          ComponentConstants.getStabilizerPose(Rotation2d.fromRotations(stabilizer.getSetpoint()))
        });
    Logger.log("RobotState/SpeedLimit", speedLimit.getAsDouble());
    Logger.log(
        "RobotState/PointedAtGoal",
        ShootingHelper.pointedAtGoal(activeShotParameters.get().actualDistance()));
    Logger.log(
        "Simulation/FieldNotes",
        NoteVisualizer.getActiveNotes(
            drivetrain.getPose(), conveyor.getGoal() == ConveyorGoal.INTAKE));
    Logger.log("Simulation/StagedNote", NoteVisualizer.getStagedNote());
  }

  public static void telemeterizeDrivetrain(SwerveDriveState state) {
    Logger.log("RobotState/RobotPose", state.Pose);
    Logger.log("RobotState/Subsystems/Swerve/MeasuredStates", state.ModuleStates);
    Logger.log("RobotState/Subsystems/Swerve/SetpointStates", state.ModuleTargets);
  }

  public static void telemeterizeMechanisms(Mechanism2d measured, Mechanism2d setpoint) {
    Logger.log("RobotState/Mechanisms/Measured", measured);
    Logger.log("RobotState/Mechanisms/Setpoint", setpoint);
  }
}
