// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc2024;

import java.util.HashMap;
import java.util.ResourceBundle.Control;
import java.util.function.Supplier;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.hardware.CANcoder;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.hardware.TalonFX;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule.DriveRequestType;
import com.fasterxml.jackson.databind.util.RootNameLookup;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.path.PathConstraints;
import com.pathplanner.lib.path.PathPlannerPath;
import com.team4522.lib.util.PolygonalPoseArea;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Transform2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.ConditionalCommand;
import edu.wpi.first.wpilibj2.command.InstantCommand;
import edu.wpi.first.wpilibj2.command.Subsystem;
import edu.wpi.first.wpilibj2.command.Command.InterruptionBehavior;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import edu.wpi.first.wpilibj2.command.button.Trigger;
import frc2024.constants.ElevatorConstants;
import frc2024.constants.FieldConstants;
import frc2024.constants.PivotConstants;
import frc2024.constants.ShooterConstants;
import frc2024.constants.SwerveConstants;
import frc2024.controlboard.Controlboard;
import frc2024.subsystems.Elevator;
import frc2024.subsystems.Pivot;
import frc2024.subsystems.Shooter;
import frc2024.subsystems.swerve.Drivetrain;
import frc2024.subsystems.swerve.generated.TunerConstants;
import lombok.Getter;

public class RobotContainer {

  public record Subsystems(Drivetrain drivetrain, Elevator elevator, Pivot pivot, Shooter shooter){}

  private static final Drivetrain drivetrain = TunerConstants.DriveTrain;
  private static final Elevator elevator = new Elevator(ElevatorConstants.ELEVATOR_CONSTANTS);
  private static final Pivot pivot = new Pivot(PivotConstants.PIVOT_CONSTANTS);
  private static final Shooter shooter = new Shooter(ShooterConstants.SHOOTER_CONSTANTS);

  @Getter
  private static final Subsystems subsystems = new Subsystems(drivetrain, elevator, pivot, shooter);

  @Getter
  private static final RobotState robotState = new RobotState(subsystems);

  public RobotContainer() {
    configButtonBindings();
    configDefaultCommands();

    if (Utils.isSimulation()) {
      drivetrain.seedFieldRelative(new Pose2d(FieldConstants.FIELD_DIMENSIONS.div(2.0), Rotation2d.fromDegrees(0)));
    }
    drivetrain.registerTelemetry(robotState::telemeterizeDrivetrain);
  }

  private void configButtonBindings() {
    Controlboard.driveController.b().whileTrue(drivetrain.applyRequest(() -> drivetrain.getUtil().getFacingAngle(Controlboard.getTranslation().get(), robotState.getShotParameters().targetHeading())));
    Controlboard.driveController
      .a()
        .whileTrue(drivetrain.applyRequest(() -> drivetrain.getUtil().getFacingAngle(Controlboard.getTranslation().get(), Rotation2d.fromDegrees(90))));
    Controlboard.driveController
      .a()
        .and(() -> drivetrain.getWithinAngleThreshold(Rotation2d.fromDegrees(90), Rotation2d.fromDegrees(30)))
          .whileTrue(elevator.setGoalCommand(Elevator.Goal.AMP).alongWith(pivot.setGoalCommand(Pivot.Goal.AMP)));
    Controlboard.driveController.leftBumper().whileTrue(AutoBuilder.pathfindToPose(new Pose2d(1.83, 7.75, Rotation2d.fromDegrees(90)), new PathConstraints(10, 10, 100, 100)));
    Controlboard.driveController
      .y()
        .whileTrue(drivetrain.applyRequest(() -> drivetrain.getUtil().getFacingAngle(Controlboard.getTranslation().get(), Rotation2d.fromDegrees(0)))
          .alongWith(elevator.setGoalCommand(Elevator.Goal.SUB).alongWith(pivot.setGoalCommand(Pivot.Goal.SUB)).alongWith(shooter.setGoalCommand(Shooter.Goal.SUB))));
  }

  private void configDefaultCommands(){
    drivetrain.setDefaultCommand(
      drivetrain.applyRequest(
        () -> Controlboard.getFieldCentric().getAsBoolean() 
          ? drivetrain.getUtil().getFieldCentric(Controlboard.getTranslation().get(), Controlboard.getRotation().getAsDouble()) 
          : drivetrain.getUtil().getRobotCentric(Controlboard.getTranslation().get(), Controlboard.getRotation().getAsDouble())));

    elevator.setDefaultCommand(elevator.setGoalCommand(Elevator.Goal.AUTO));
    pivot.setDefaultCommand(pivot.setGoalCommand(Pivot.Goal.AIM));
    shooter.setDefaultCommand(shooter.setGoalCommand(Shooter.Goal.AUTO));
  }

  public Command getAutonomousCommand() {
    return AutoBuilder.buildAuto("ExampleAuto");
  }
}
