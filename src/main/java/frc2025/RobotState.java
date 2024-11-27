package frc2025;

import com.ctre.phoenix6.swerve.SwerveDrivetrain.SwerveDriveState;
import dashboard.Mechanism;
import dashboard.MechanismVisualizer;
import data.DataConversions;
import drivers.TalonFXSubsystem.TalonFXSubsystemGoal;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import frc2025.RobotContainer.Subsystems;
import frc2025.commands.ShootSimNote;
import frc2025.constants.FieldConstants;
import frc2025.constants.SimConstants;
import frc2025.controlboard.Controlboard;
import frc2025.logging.ComponentConstants;
import frc2025.logging.Logger;
import frc2025.logging.NoteVisualizer;
import frc2025.subsystems.conveyor.Conveyor;
import frc2025.subsystems.drivetrain.Drivetrain;
import frc2025.subsystems.elevator.Elevator;
import frc2025.subsystems.elevator.Elevator.ElevatorGoal;
import frc2025.subsystems.intake.Intake;
import frc2025.subsystems.intake.Intake.IntakeGoal;
import frc2025.subsystems.pivot.Pivot;
import frc2025.subsystems.pivot.Pivot.PivotGoal;
import frc2025.subsystems.shooter.Shooter;
import frc2025.subsystems.shooter.Shooter.ShooterGoal;
import frc2025.subsystems.stabilizer.Stabilizer;
import frc2025.subsystems.stabilizer.StabilizerConstants;
import frc2025.subsystems.vision.Vision;
import frc2025.util.ShootingHelper;
import frc2025.util.ShootingHelper.ShotParameters;
import java.util.ArrayList;
import java.util.Collections;
import java.util.HashSet;
import java.util.Set;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
import lombok.Getter;
import util.AllianceFlipUtil;
import util.GeomUtil;

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
    HOME_INTAKE(ElevatorGoal.HOME, PivotGoal.HOME_INTAKE, ShooterGoal.TRACKING),
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

  public static final ArrayList<Pose3d> activeNotes =
      new ArrayList<Pose3d>(Collections.nCopies(SimConstants.MAX_SIM_NOTES, Pose3d.kZero));

  private final Mechanism elevatorMech;

  private final Mechanism pivotMech;

  private final MechanismVisualizer mechanismVisualizer;

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

    pivotMech =
        new Mechanism("Pivot Front", pivot.getLigaments())
            .withDynamicPosition(
                () ->
                    pivotRootPosition
                        .get()
                        .plus(new Translation2d(SimConstants.MECH_ELEVATOR_X, 0)));

    elevatorMech =
        new Mechanism("Elevator", elevator.getLigament())
            .withStaticPosition(new Translation2d(SimConstants.MECH_ELEVATOR_X, 0));

    mechanismVisualizer =
        new MechanismVisualizer(
            SimConstants.MEASURED_MECHANISM,
            SimConstants.SETPOINT_MECHANISM,
            RobotState::telemeterizeMechanisms,
            elevatorMech,
            pivotMech);

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
        .withName("applySuperstructureGoal(" + goal.toString() + ")");
  }

  public static ShotParameters getActiveShotParameters() {
    return ShootingHelper.calculateSimpleShotParameters(
        drivetrain.getPose().getTranslation(), getActiveSpeaker().get().toTranslation2d());
  }

  public static DoubleSupplier getSpeedLimit() {
    return () -> {
      if (ShootingHelper.withinRange()
          && Controlboard.driveController.getHID().getLeftBumperButton()) {
        return MathUtil.clamp(
            getActiveShotParameters().actualDistance().div(7).getMeters(), 0.5, 1);
      } else if (Controlboard.driveController.getLeftTriggerAxis()
          > Controlboard.TRIGGER_DEADBAND) {
        return 0.5;
      } else {
        return 1;
      }
    };
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
    logShotParameters(getActiveShotParameters());
    Logger.log(
        "ActiveCommands", activeCommands.stream().map(Command::getName).toArray(String[]::new));
    Logger.log(
        "RobotState/Components/MeasuredComponents",
        new Pose3d[] {
          ComponentConstants.getElevStage1Pose(elevator.getMeasuredHeight().getMeters()),
          ComponentConstants.getElevStage2Pose(elevator.getMeasuredHeight().getMeters()),
          ComponentConstants.getShooterPose(
              elevator.getMeasuredHeight().getMeters(), pivot.getAngle()),
          ComponentConstants.getStabilizerPose(
              stabilizer.getAngle().times(StabilizerConstants.REAL_POS_TO_SIM_SCALAR))
        });
    Logger.log(
        "RobotState/Components/SetpointComponents",
        new Pose3d[] {
          ComponentConstants.getElevStage1Pose(elevator.getSetpointHeight().getMeters()),
          ComponentConstants.getElevStage2Pose(elevator.getSetpointHeight().getMeters()),
          ComponentConstants.getShooterPose(
              elevator.getSetpointHeight().getMeters(),
              Rotation2d.fromRotations(pivot.getSetpoint())),
          ComponentConstants.getStabilizerPose(
              Rotation2d.fromRotations(stabilizer.getSetpoint())
                  .times(StabilizerConstants.REAL_POS_TO_SIM_SCALAR))
        });
    Logger.log("RobotState/SpeedLimit", getSpeedLimit().getAsDouble());
    Logger.log(
        "RobotState/PointedAtGoal",
        ShootingHelper.pointedAtGoal(getActiveShotParameters().actualDistance()));
    (Robot.isSimulation()
            ? NoteVisualizer.getClosestNote(drivetrain.getPose())
            : Vision.getNotePose(drivetrain.getPose()))
        .ifPresent(
            translation ->
                Logger.log(
                    "RobotState/VisibleNotePose",
                    GeomUtil.translationToPose3d(
                        translation, FieldConstants.NOTE_HEIGHT.getMeters())));
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

  private static void logShotParameters(ShotParameters shotParameters) {
    Logger.log("RobotState/ActiveShotParameters/ActualDistance", shotParameters.actualDistance());
    Logger.log(
        "RobotState/ActiveShotParameters/EffectiveDistance", shotParameters.effectiveDistance());
    Logger.log(
        "RobotState/ActiveShotParameters/TargetHeading",
        shotParameters.targetHeading().getDegrees());
    Logger.log("RobotState/ActiveShotParameters/Lookahead", shotParameters.shotLookahead());
    Logger.log(
        "RobotState/ActiveShotParameters/ShootState/ElevatorHeight",
        shotParameters.shootState().getElevatorHeight());
    Logger.log(
        "RobotState/ActiveShotParameters/ShootState/PivotAngle",
        shotParameters.shootState().getPivotAngleDeg());
    Logger.log(
        "RobotState/ActiveShotParameters/ShootState/Velocity",
        shotParameters.shootState().getVelocityRPM());
  }
}
