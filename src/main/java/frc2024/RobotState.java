package frc2024;

import com.SCREAMLib.data.DataConversions;
import com.SCREAMLib.data.Length;
import com.SCREAMLib.drivers.TalonFXSubsystem.TalonFXSubsystemGoal;
import com.SCREAMLib.util.AllianceFlipUtil;
import com.ctre.phoenix6.mechanisms.swerve.CustomSwerveDrivetrain.SwerveDriveState;
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
import frc2024.subsystems.elevator.Elevator;
import frc2024.subsystems.elevator.Elevator.ElevatorGoal;
import frc2024.subsystems.elevator.ElevatorConstants;
import frc2024.subsystems.intake.Intake;
import frc2024.subsystems.intake.Intake.IntakeGoal;
import frc2024.subsystems.pivot.Pivot;
import frc2024.subsystems.pivot.Pivot.PivotGoal;
import frc2024.subsystems.shooter.Shooter;
import frc2024.subsystems.shooter.Shooter.ShooterGoal;
import frc2024.subsystems.shooter.ShootingHelper;
import frc2024.subsystems.shooter.ShootingHelper.ShotParameters;
import frc2024.subsystems.stabilizer.Stabilizer;
import frc2024.subsystems.swerve.Drivetrain;
import frc2024.subsystems.vision.Vision;
import java.util.ArrayList;
import java.util.Collections;
import java.util.HashSet;
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
  private static Intake intake;
  private static Stabilizer stabilizer;
  private static Vision vision;

  public enum SuperstructureGoal {
    IDLE(ElevatorGoal.TRACKING, PivotGoal.TRACKING, ShooterGoal.TRACKING),
    HOME_INTAKE(ElevatorGoal.HOME_INTAKE, PivotGoal.HOME_INTAKE, ShooterGoal.TRACKING),
    TRAP_INTAKE(ElevatorGoal.TRAP_INTAKE, PivotGoal.TRAP_INTAKE, ShooterGoal.TRACKING),
    SUB(ElevatorGoal.SUB, PivotGoal.SUB, ShooterGoal.SUB),
    SUB_DEFENDED(ElevatorGoal.SUB_DEFENDED, PivotGoal.SUB_DEFENDED, ShooterGoal.SUB),
    AMP(ElevatorGoal.AMP, PivotGoal.AMP, ShooterGoal.TRACKING),
    TRAP(ElevatorGoal.TRAP, PivotGoal.TRAP, ShooterGoal.IDLE),
    PASSING(ElevatorGoal.TRACKING, PivotGoal.TRACKING, ShooterGoal.TRACKING),
    TRACKING(ElevatorGoal.TRACKING, PivotGoal.TRACKING, ShooterGoal.TRACKING),
    EJECT(ElevatorGoal.EJECT, PivotGoal.EJECT, ShooterGoal.TRACKING);

    private TalonFXSubsystemGoal[] goals;

    private SuperstructureGoal(
        ElevatorGoal elevatorGoal, PivotGoal pivotGoal, ShooterGoal shooterGoal) {
      this.goals = new TalonFXSubsystemGoal[] {elevatorGoal, pivotGoal, shooterGoal};
    }
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

  public static final ArrayList<Pose3d> activeNotes =
      new ArrayList<Pose3d>(Collections.nCopies(SimConstants.MAX_SIM_NOTES, new Pose3d()));

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

  private static final Set<Command> activeCommands = new HashSet<>();

  public RobotState(Subsystems subsystems) {
    drivetrain = subsystems.drivetrain();
    elevator = subsystems.elevator();
    pivot = subsystems.pivot();
    shooter = subsystems.shooter();
    conveyor = subsystems.conveyor();
    intake = subsystems.intake();
    stabilizer = subsystems.stabilizer();
    vision = subsystems.vision();

    mechanismVisualizer.setEnabled(true);
  }

  public static Command shootSimNoteCommand() {
    return Commands.runOnce(() -> new ShootSimNote(RobotContainer.getSubsystems()).schedule());
  }

  public Command applySuperstructureGoal(SuperstructureGoal goal) {
    return Commands.defer(
            () -> {
              activeSuperstructureGoal = goal;
              return Commands.parallel(
                  elevator.applyGoal(goal.goals[0]),
                  pivot.applyGoal(goal.goals[1]),
                  shooter.applyGoal(goal.goals[2]));
            },
            Set.of(elevator, pivot, shooter))
        .withName("RobotState: applySuperstructureGoal(" + goal.toString() + ")");
  }

  public static void addActiveCommand(Command command) {
    activeCommands.add(command);
  }

  public static void removeActiveCommand(Command command) {
    activeCommands.remove(command);
  }

  public static void outputTelemetry() {
    if (Robot.isSimulation()) {
      Logger.log("Simulation/ActiveNotes", activeNotes.toArray(Pose3d[]::new));
      Logger.log(
          "Simulation/FieldNotes",
          NoteVisualizer.getActiveNotes(
              drivetrain.getPose(), intake.getGoal() == IntakeGoal.INTAKE));
      Logger.log("Simulation/StagedNote", NoteVisualizer.getStagedNote());
    }
    Logger.log(
        "ActiveCommands", activeCommands.stream().map(Command::getName).toArray(String[]::new));
    Logger.log(
        "RobotState/HorizontalDistanceFromGoal",
        getActiveShotParameters().get().effectiveDistance());
    Logger.log(
        "RobotState/Components/MeasuredComponents",
        new Pose3d[] {
          ComponentConstants.getElevStage1Pose(elevator.getMeasuredHeight().getMeters()),
          ComponentConstants.getElevStage2Pose(elevator.getMeasuredHeight().getMeters()),
          ComponentConstants.getShooterPose(
              elevator.getMeasuredHeight().getMeters(), pivot.getAngle()),
          ComponentConstants.getStabilizerPose(stabilizer.getAngle())
        });
    Logger.log(
        "RobotState/Components/SetpointComponents",
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
  }

  public static void telemeterizeDrivetrain(SwerveDriveState state) {
    Logger.log("RobotState/RobotPose", state.Pose);
    Logger.log("RobotState/Subsystems/Drivetrain/MeasuredStates", state.ModuleStates);
    Logger.log("RobotState/Subsystems/Drivetrain/SetpointStates", state.ModuleTargets);
  }

  public static void telemeterizeMechanisms(Mechanism2d measured, Mechanism2d setpoint) {
    Logger.log("RobotState/Mechanisms/Measured", measured);
    Logger.log("RobotState/Mechanisms/Setpoint", setpoint);
  }
}
