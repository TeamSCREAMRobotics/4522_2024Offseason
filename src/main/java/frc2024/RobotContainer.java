// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc2024;

import com.SCREAMLib.util.AllianceFlipUtil;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc2024.RobotState.SuperstructureGoal;
import frc2024.constants.FieldConstants;
import frc2024.controlboard.Controlboard;
import frc2024.subsystems.conveyor.Conveyor;
import frc2024.subsystems.conveyor.Conveyor.ConveyorGoal;
import frc2024.subsystems.conveyor.ConveyorConstants;
import frc2024.subsystems.elevator.Elevator;
import frc2024.subsystems.elevator.Elevator.ElevatorGoal;
import frc2024.subsystems.elevator.ElevatorConstants;
import frc2024.subsystems.pivot.Pivot;
import frc2024.subsystems.pivot.Pivot.PivotGoal;
import frc2024.subsystems.pivot.PivotConstants;
import frc2024.subsystems.shooter.Shooter;
import frc2024.subsystems.shooter.ShooterConstants;
import frc2024.subsystems.shooter.ShootingHelper;
import frc2024.subsystems.stabilizer.Stabilizer;
import frc2024.subsystems.stabilizer.Stabilizer.StabilizerGoal;
import frc2024.subsystems.stabilizer.StabilizerConstants;
import frc2024.subsystems.swerve.Drivetrain;
import frc2024.subsystems.swerve.SwerveConstants;
import frc2024.subsystems.swerve.generated.TunerConstants;
import frc2024.subsystems.vision.Vision;
import java.util.Optional;
import lombok.Getter;

public class RobotContainer {

  public record Subsystems(
      Drivetrain drivetrain,
      Elevator elevator,
      Pivot pivot,
      Shooter shooter,
      Conveyor conveyor,
      Stabilizer stabilizer,
      Vision vision) {}

  private static final Drivetrain drivetrain = TunerConstants.DriveTrain;
  private static final Elevator elevator = new Elevator(ElevatorConstants.SUBSYSTEM_CONSTANTS);
  private static final Pivot pivot = new Pivot(PivotConstants.SUBSYSTEM_CONSTANTS);
  private static final Shooter shooter = new Shooter(ShooterConstants.SUBSYSTEM_CONSTANTS);
  private static final Conveyor conveyor = new Conveyor(ConveyorConstants.SUBSYSTEM_CONSTANTS);
  private static final Stabilizer stabilizer =
      new Stabilizer(StabilizerConstants.SUBSYSTEM_CONSTANTS);
  private static final Vision vision = new Vision();

  @Getter
  private static final Subsystems subsystems =
      new Subsystems(drivetrain, elevator, pivot, shooter, conveyor, stabilizer, vision);

  @Getter private static final RobotState robotState = new RobotState(subsystems);

  public RobotContainer() {
    NamedCommands.registerCommand(
        "Shoot", RobotState.shootSimNoteCommand().onlyIf(conveyor.hasNote()));
    NamedCommands.registerCommand(
        "Intake", robotState.applySuperstructureGoal(SuperstructureGoal.HOME_INTAKE));
    configDefaultCommands();
    configButtonBindings();

    if (Robot.isSimulation()) {
      drivetrain.seedFieldRelative(
          new Pose2d(FieldConstants.FIELD_DIMENSIONS.div(2), Rotation2d.fromDegrees(0)));
    }

    drivetrain.registerTelemetry(RobotState::telemeterizeDrivetrain);
  }

  private void configButtonBindings() {
    Controlboard.driveController.back().onTrue(Commands.runOnce(() -> drivetrain.resetHeading()));

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
                            RobotState.getActiveShotParameters().get().targetHeading())));

    /* Controlboard.driveController
        .a()
        .whileTrue(
            drivetrain
                .applyRequest(
                    () ->
                        drivetrain
                            .getHelper()
                            .getFacingAngleProfiled(
                                Controlboard.getTranslation().get(),
                                Rotation2d.fromDegrees(90),
                                SwerveConstants.HEADING_CONTROLLER))
                .beforeStarting(
                    () ->
                        SwerveConstants.HEADING_CONTROLLER.reset(
                            drivetrain.getHeading().getRadians())));

    Controlboard.driveController
        .a()
        .and(
            () ->
                drivetrain.getWithinAngleThreshold(
                    Rotation2d.fromDegrees(90), Rotation2d.fromDegrees(45)))
        .whileTrue(robotState.applySuperstructureGoal(SuperstructureGoal.AMP)); */

    Controlboard.driveController
        .a()
        .whileTrue(
            AutoBuilder.pathfindThenFollowPath(
                PathPlannerPath.fromPathFile("To Amp"), new PathConstraints(5.0, 7.5, 8.0, 8.0)))
        .and(
            () ->
                drivetrain
                        .getPose()
                        .getTranslation()
                        .getDistance(new Translation2d(1.8, FieldConstants.FIELD_DIMENSIONS.getY()))
                    < 2.0)
        .whileTrue(robotState.applySuperstructureGoal(SuperstructureGoal.AMP));

    Controlboard.driveController
        .rightTrigger()
        .whileTrue(
            robotState
                .applySuperstructureGoal(SuperstructureGoal.HOME_INTAKE)
                .alongWith(conveyor.applyGoal(ConveyorGoal.INTAKE)));

    Controlboard.driveController
        .x()
        .toggleOnTrue(
            elevator.runVoltage(
                () ->
                    -MathUtil.applyDeadband(Controlboard.driveController.getRightY(), 0.05) * 12));

    Controlboard.driveController
        .rightBumper()
        .and(
            () ->
                ShootingHelper.validShot(
                    RobotState.getActiveShotParameters().get().effectiveDistance()))
        .and(() -> Robot.isSimulation())
        .whileTrue(conveyor.applyGoal(ConveyorGoal.INTAKE))
        .and(conveyor.hasNote())
        .whileTrue(RobotState.shootSimNoteCommand());

    Controlboard.driveController
        .povRight()
        .whileTrue(robotState.applySuperstructureGoal(SuperstructureGoal.EJECT));

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
                                AllianceFlipUtil.getForwardRotation(),
                                SwerveConstants.HEADING_CONTROLLER))
                .beforeStarting(
                    () ->
                        SwerveConstants.HEADING_CONTROLLER.reset(
                            drivetrain.getHeading().getRadians()))
                .alongWith(robotState.applySuperstructureGoal(SuperstructureGoal.SUB)));

    Controlboard.driveController
        .b()
        .toggleOnTrue(
            stabilizer
                .applyGoal(StabilizerGoal.OUT)
                .alongWith(elevator.applyGoal(ElevatorGoal.TRAP))
                .alongWith(pivot.applyGoal(PivotGoal.TRAP)));
  }

  private void configDefaultCommands() {
    drivetrain.setDefaultCommand(
        drivetrain.applyRequest(
            () ->
                Controlboard.getFieldCentric().getAsBoolean()
                    ? drivetrain
                        .getHelper()
                        .getFieldCentric(
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
                            Controlboard.getRotation().getAsDouble())));
  }

  double startTime = 0;

  private void startTime() {
    startTime = Timer.getFPGATimestamp();
  }

  public Command getAutonomousCommand() {
    PathPlannerPath path = PathPlannerPath.fromPathFile("6PieceNuggets");
    return new SequentialCommandGroup(
            Commands.runOnce(
                () ->
                    drivetrain.seedFieldRelative(
                        AllianceFlipUtil.MirroredPose2d(path.getPreviewStartingHolonomicPose()))),
            Commands.waitUntil(() -> pivot.atGoal()),
            Commands.runOnce(() -> startTime()),
            AutoBuilder.followPath(path)
                .deadlineWith(
                    Commands.run(
                        () ->
                            PPHolonomicDriveController.setRotationTargetOverride(
                                () ->
                                    Optional.of(
                                        RobotState.getActiveShotParameters()
                                            .get()
                                            .targetHeading())))),
            Commands.runOnce(() -> System.out.println(Timer.getFPGATimestamp() - startTime)))
        .alongWith(conveyor.applyGoal(ConveyorGoal.INTAKE));
  }
}
