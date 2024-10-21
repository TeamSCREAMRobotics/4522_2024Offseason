// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc2025;

import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.PathPlannerPath;
import drivers.TalonFXSubsystem;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc2025.RobotState.SuperstructureGoal;
import frc2025.constants.FieldConstants;
import frc2025.controlboard.Controlboard;
import frc2025.logging.NoteVisualizer;
import frc2025.subsystems.conveyor.Conveyor;
import frc2025.subsystems.conveyor.Conveyor.ConveyorGoal;
import frc2025.subsystems.conveyor.ConveyorConstants;
import frc2025.subsystems.drivetrain.Drivetrain;
import frc2025.subsystems.drivetrain.DrivetrainConstants;
import frc2025.subsystems.drivetrain.generated.TunerConstants;
import frc2025.subsystems.elevator.Elevator;
import frc2025.subsystems.elevator.ElevatorConstants;
import frc2025.subsystems.intake.Intake;
import frc2025.subsystems.intake.Intake.IntakeGoal;
import frc2025.subsystems.intake.IntakeConstants;
import frc2025.subsystems.pivot.Pivot;
import frc2025.subsystems.pivot.PivotConstants;
import frc2025.subsystems.shooter.Shooter;
import frc2025.subsystems.shooter.ShooterConstants;
import frc2025.subsystems.stabilizer.Stabilizer;
import frc2025.subsystems.stabilizer.Stabilizer.StabilizerGoal;
import frc2025.subsystems.stabilizer.StabilizerConstants;
import frc2025.subsystems.vision.Vision;
import frc2025.util.ShootingHelper;
import java.util.Optional;
import lombok.Getter;
import util.AllianceFlipUtil;
import util.PPUtil;

public class RobotContainer {

  public record Subsystems(
      Drivetrain drivetrain,
      Elevator elevator,
      Pivot pivot,
      Shooter shooter,
      Conveyor conveyor,
      Intake intake,
      Stabilizer stabilizer,
      Vision vision) {}

  private static final Drivetrain drivetrain = TunerConstants.DriveTrain;
  private static final Elevator elevator = new Elevator(ElevatorConstants.SUBSYSTEM_CONSTANTS);
  private static final Pivot pivot = new Pivot(PivotConstants.SUBSYSTEM_CONSTANTS);
  private static final Shooter shooter = new Shooter(ShooterConstants.SUBSYSTEM_CONSTANTS);
  private static final Conveyor conveyor = new Conveyor(ConveyorConstants.SUBSYSTEM_CONSTANTS);
  private static final Intake intake = new Intake(IntakeConstants.SUBSYSTEM_CONSTANTS);
  private static final Stabilizer stabilizer =
      new Stabilizer(StabilizerConstants.SUBSYSTEM_CONSTANTS);
  private static final Vision vision = new Vision();

  @Getter
  private static final Subsystems subsystems =
      new Subsystems(drivetrain, elevator, pivot, shooter, conveyor, intake, stabilizer, vision);

  @Getter private static final RobotState robotState = new RobotState(subsystems);

  public RobotContainer() {
    NamedCommands.registerCommand(
        "Shoot", RobotState.shootSimNoteCommand().onlyIf(conveyor.hasNote()));
    NamedCommands.registerCommand(
        "Intake", robotState.applySuperstructureGoal(SuperstructureGoal.HOME_INTAKE));
    NamedCommands.registerCommand("StartAiming", startAiming());
    NamedCommands.registerCommand("StopAiming", stopAiming());

    if (Robot.isSimulation()) {
      drivetrain.seedFieldRelative(
          new Pose2d(FieldConstants.FIELD_DIMENSIONS.div(2), Rotation2d.fromDegrees(0)));
    }

    drivetrain.registerTelemetry(RobotState::telemeterizeDrivetrain);

    configDefaultCommands();
    configButtonBindings();
  }

  private void configButtonBindings() {
    Controlboard.driveController
        .back()
        .onTrue(
            Commands.runOnce(
                () -> {
                  drivetrain.resetHeading();
                  drivetrain.getHelper().setLastAngle(drivetrain.getHeading());
                }));
    Controlboard.driveController
        .start()
        .and(() -> Robot.isSimulation())
        .onTrue(
            Commands.runOnce(() -> drivetrain.updateFromVision(Vision.getRandomPoseEstimate()))
                .ignoringDisable(true));

    Controlboard.driveController
        .leftBumper()
        .whileTrue(
            drivetrain.applyRequest(
                () ->
                    drivetrain
                        .getHelper()
                        .getFacingAngle(
                            Controlboard.getTranslation()
                                .get()
                                .times(RobotState.getSpeedLimit().getAsDouble()),
                            RobotState.getActiveShotParameters().targetHeading())));

    Controlboard.driveController
        .a()
        .whileTrue(
            drivetrain
                .applyRequest(
                    () ->
                        drivetrain
                            .getHelper()
                            .getFacingAngleProfiled(
                                Controlboard.getTranslation()
                                    .get()
                                    .times(RobotState.getSpeedLimit().getAsDouble()),
                                Rotation2d.fromDegrees(90),
                                DrivetrainConstants.HEADING_CONTROLLER))
                .beforeStarting(
                    () ->
                        DrivetrainConstants.HEADING_CONTROLLER.reset(
                            drivetrain.getHeading().getRadians())));

    Controlboard.driveController
        .a()
        .and(
            () ->
                drivetrain.getWithinAngleThreshold(
                    Rotation2d.fromDegrees(90), Rotation2d.fromDegrees(45)))
        .whileTrue(robotState.applySuperstructureGoal(SuperstructureGoal.AMP));

    Controlboard.driveController
        .rightTrigger()
        .and(new Trigger(conveyor.hasNote()).negate())
        .whileTrue(
            drivetrain.getNoteAssistCommand(
                () ->
                    Controlboard.getTranslation()
                        .get()
                        .times(RobotState.getSpeedLimit().getAsDouble()),
                Controlboard.getRotation(),
                () ->
                    Robot.isSimulation()
                        ? NoteVisualizer.getClosestNote(drivetrain.getPose())
                        : Vision.getNotePose(drivetrain.getPose())))
        .whileTrue(
            robotState
                .applySuperstructureGoal(SuperstructureGoal.HOME_INTAKE)
                .alongWith(conveyor.applyGoal(ConveyorGoal.INTAKE))
                .alongWith(intake.applyGoal(IntakeGoal.INTAKE)));

    Controlboard.driveController
        .x()
        .toggleOnTrue(
            elevator.applyVoltage(
                () -> -MathUtil.applyDeadband(Controlboard.driveController.getRightY(), 0.05) * 3,
                () -> ElevatorConstants.SUBSYSTEM_CONSTANTS.slot0.kG));

    Controlboard.driveController
        .rightBumper()
        .and(Controlboard.driveController.leftBumper())
        .and(conveyor.hasNote())
        .and(
            () ->
                ShootingHelper.validShot(RobotState.getActiveShotParameters().effectiveDistance()))
        .whileTrue(RobotState.shootSimNoteCommand());

    Controlboard.driveController
        .rightBumper()
        .and(Controlboard.driveController.leftBumper())
        .and(
            () ->
                ShootingHelper.validShot(RobotState.getActiveShotParameters().effectiveDistance()))
        .whileTrue(
            conveyor.applyGoal(ConveyorGoal.SHOOT).alongWith(intake.applyGoal(IntakeGoal.INTAKE)));

    Controlboard.driveController
        .povRight()
        .whileTrue(
            robotState
                .applySuperstructureGoal(SuperstructureGoal.EJECT)
                .alongWith(
                    conveyor.applyGoal(ConveyorGoal.EJECT), intake.applyGoal(IntakeGoal.EJECT)));

    Controlboard.driveController
        .y()
        .whileTrue(
            drivetrain
                .applyRequest(
                    () ->
                        drivetrain
                            .getHelper()
                            .getFacingAngleProfiled(
                                Controlboard.getTranslation().get(),
                                AllianceFlipUtil.getFwdHeading(),
                                DrivetrainConstants.HEADING_CONTROLLER))
                .beforeStarting(
                    () ->
                        DrivetrainConstants.HEADING_CONTROLLER.reset(
                            drivetrain.getHeading().getRadians())))
        .whileTrue(robotState.applySuperstructureGoal(SuperstructureGoal.SUB));

    Controlboard.driveController
        .b()
        .toggleOnTrue(stabilizer.applyGoal(StabilizerGoal.OUT))
        .toggleOnTrue(robotState.applySuperstructureGoal(SuperstructureGoal.TRAP));
  }

  private void configDefaultCommands() {
    drivetrain.setDefaultCommand(
        drivetrain
            .applyRequest(
                () ->
                    Controlboard.getFieldCentric().getAsBoolean()
                        ? drivetrain
                            .getHelper()
                            .getHeadingCorrectedFieldCentric(
                                Controlboard.getTranslation()
                                    .get()
                                    .times(RobotState.getSpeedLimit().getAsDouble()),
                                Controlboard.getRotation().getAsDouble())
                        : drivetrain
                            .getHelper()
                            .getRobotCentric(
                                Controlboard.getTranslation()
                                    .get()
                                    .times(RobotState.getSpeedLimit().getAsDouble()),
                                Controlboard.getRotation().getAsDouble()))
            .beforeStarting(() -> drivetrain.getHelper().setLastAngle(drivetrain.getHeading()))
            .withName("Drivetrain: Default command"));
  }

  private void stopAll() {
    drivetrain.stop();
    TalonFXSubsystem.stopAll(elevator, pivot, shooter, conveyor, intake, stabilizer);
  }

  double startTime = 0;

  private void startTime() {
    startTime = Timer.getFPGATimestamp();
  }

  public Command getAutonomousCommand() {
    Optional<PathPlannerPath> path = PPUtil.loadPathFile("7PieceNuggets");
    if (path.isEmpty()) {
      return Commands.none();
    }
    return new SequentialCommandGroup(
            Commands.runOnce(
                () ->
                    drivetrain.seedFieldRelative(
                        AllianceFlipUtil.MirroredPose2d(
                            new Pose2d(
                                path.get().getPoint(0).position,
                                AllianceFlipUtil.getFwdHeading())))),
            Commands.waitUntil(() -> pivot.atGoal()),
            Commands.runOnce(() -> startTime()),
            RobotState.shootSimNoteCommand(),
            startAiming(),
            AutoBuilder.followPath(path.get()),
            Commands.runOnce(() -> System.out.println(Timer.getFPGATimestamp() - startTime)),
            Commands.runOnce(() -> stopAll()))
        .alongWith(intake.applyGoal(IntakeGoal.INTAKE), conveyor.applyGoal(ConveyorGoal.INTAKE));
  }

  public Command startAiming() {
    return Commands.runOnce(
        () ->
            PPHolonomicDriveController.overrideRotationFeedback(
                drivetrain.calculatePathRotationFeedback(
                    RobotState.getActiveShotParameters().targetHeading())));
  }

  public Command stopAiming() {
    return Commands.runOnce(() -> PPHolonomicDriveController.clearRotationFeedbackOverride());
  }
}
