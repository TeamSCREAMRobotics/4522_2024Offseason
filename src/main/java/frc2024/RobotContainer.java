// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc2024;

import java.util.Optional;
import java.util.function.Supplier;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.auto.NamedCommands;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.team4522.lib.util.AllianceFlipUtil;
import com.team4522.lib.util.FeedForwardCharacterization;
import com.team4522.lib.util.ScreamUtil;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.PrintCommand;
import edu.wpi.first.wpilibj2.command.SequentialCommandGroup;
import frc2024.RobotState.SuperstructureGoal;
import frc2024.commands.ShootSimNote;
import frc2024.constants.FieldConstants;
import frc2024.controlboard.Controlboard;
import frc2024.subsystems.conveyor.Conveyor;
import frc2024.subsystems.conveyor.ConveyorConstants;
import frc2024.subsystems.elevator.Elevator;
import frc2024.subsystems.elevator.ElevatorConstants;
import frc2024.subsystems.pivot.Pivot;
import frc2024.subsystems.pivot.PivotConstants;
import frc2024.subsystems.pivot.Pivot.Goal;
import frc2024.subsystems.shooter.Shooter;
import frc2024.subsystems.shooter.ShooterConstants;
import frc2024.subsystems.shooter.ShootingUtils;
import frc2024.subsystems.stabilizers.StabilizerConstants;
import frc2024.subsystems.stabilizers.Stabilizers;
import frc2024.subsystems.swerve.Drivetrain;
import frc2024.subsystems.swerve.SwerveConstants;
import frc2024.subsystems.swerve.generated.TunerConstants;
import lombok.Getter;

public class RobotContainer {

  public record Subsystems(Drivetrain drivetrain, Elevator elevator, Pivot pivot, Shooter shooter, Conveyor conveyor, Stabilizers stabilizers){}

  private static final Drivetrain drivetrain = TunerConstants.DriveTrain;
  private static final Elevator elevator = new Elevator(ElevatorConstants.ELEVATOR_CONSTANTS);
  private static final Pivot pivot = new Pivot(PivotConstants.PIVOT_CONSTANTS);
  private static final Shooter shooter = new Shooter(ShooterConstants.SHOOTER_CONSTANTS);
  private static final Conveyor conveyor = new Conveyor(ConveyorConstants.CONVEYOR_CONSTANTS);
  private static final Stabilizers stabilizers = new Stabilizers(StabilizerConstants.STABILIZER_CONSTANTS);

  @Getter
  private static final Subsystems subsystems = new Subsystems(drivetrain, elevator, pivot, shooter, conveyor, stabilizers);

  @Getter
  private static final RobotState robotState = new RobotState(subsystems);

  public RobotContainer() {
    NamedCommands.registerCommand("Shoot", RobotState.shootSimNoteCommand());
    configButtonBindings();
    configDefaultCommands();

    if (Utils.isSimulation()) {
      drivetrain.seedFieldRelative(new Pose2d(FieldConstants.FIELD_DIMENSIONS.div(2.0), Rotation2d.fromDegrees(0)));
    }

    drivetrain.registerTelemetry(robotState::telemeterizeDrivetrain);
  }

  private void configButtonBindings() {
    Controlboard.driveController.rightBumper()
      .whileTrue(
        drivetrain.applyRequest(() -> 
          drivetrain.getUtil().getFacingAngleProfiled(
            Controlboard.getTranslation().get(), 
            drivetrain.getPose().getRotation(), 
            robotState.getActiveShotParameters().get().targetHeading(), 
            SwerveConstants.HEADING_CONTROLLER)))
      .onFalse(Commands.runOnce(() -> SwerveConstants.HEADING_CONTROLLER.reset(drivetrain.getPose().getRotation().getRadians())));

    Controlboard.driveController.a()
      .whileTrue(drivetrain.applyRequest(() -> drivetrain.getUtil().getFacingAngle(Controlboard.getTranslation().get(), Rotation2d.fromDegrees(90))));

    Controlboard.driveController.a().and(() -> drivetrain.getWithinAngleThreshold(Rotation2d.fromDegrees(90), Rotation2d.fromDegrees(30)))
      .whileTrue(robotState.setSuperstructureGoalCommand(SuperstructureGoal.AMP));

    Controlboard.driveController.rightBumper()
      .and(() -> ShootingUtils.validShot())
      .debounce(0.075)
      .onTrue(RobotState.shootSimNoteCommand());

    Controlboard.driveController.x()
      .toggleOnTrue(elevator.runVoltage(() -> (Controlboard.driveController.getRightTriggerAxis() - Controlboard.driveController.getLeftTriggerAxis()) * 12));

    Controlboard.driveController.leftBumper()
      .onTrue(RobotState.shootSimNoteCommand());

    Controlboard.driveController.povRight()
      .whileTrue(robotState.setSuperstructureGoalCommand(SuperstructureGoal.EJECT));

    Controlboard.driveController.y()
      .whileTrue(drivetrain.applyRequest(() -> drivetrain.getUtil().getFacingAngle(Controlboard.getTranslation().get(), Rotation2d.fromDegrees(0)))
      .alongWith(robotState.setSuperstructureGoalCommand(SuperstructureGoal.SUB)));

    Controlboard.driveController.b()
      .toggleOnTrue(stabilizers.setGoalCommand(Stabilizers.Goal.OUT).alongWith(elevator.moveToGoal(Elevator.Goal.TRAP)));
  }

  private void configDefaultCommands(){
    drivetrain.setDefaultCommand(
      drivetrain.applyRequest(
        () -> Controlboard.getFieldCentric().getAsBoolean()
          ? drivetrain.getUtil().getFieldCentric(Controlboard.getTranslation().get(), Controlboard.getRotation().getAsDouble()) 
          : drivetrain.getUtil().getRobotCentric(Controlboard.getTranslation().get(), Controlboard.getRotation().getAsDouble())));
  }

  double startTime = 0;
  private void startTime(){
    startTime = Timer.getFPGATimestamp();
  }

  public Command getAutonomousCommand() {
    PathPlannerPath path = PathPlannerPath.fromPathFile("Test Path");
    return new SequentialCommandGroup(
      Commands.runOnce(() -> startTime()),
      Commands.runOnce(() -> drivetrain.seedFieldRelative(path.getPreviewStartingHolonomicPose())),
      RobotState.shootSimNoteCommand(),
      AutoBuilder.followPath(path)
        .deadlineWith(Commands.run(() -> PPHolonomicDriveController.setRotationTargetOverride(() -> Optional.of(ScreamUtil.calculateAngleToPoint(drivetrain.getPose().getTranslation(), robotState.getActiveSpeaker().toTranslation2d()).plus(new Rotation2d(Math.PI)))))),
      Commands.runOnce(() -> System.out.println(Timer.getFPGATimestamp() - startTime))
    );
  }
}
