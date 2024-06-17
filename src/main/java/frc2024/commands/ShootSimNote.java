// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc2024.commands;

import com.team4522.lib.math.Conversions;
import com.team4522.lib.util.ScreamUtil;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc2024.RobotContainer;
import frc2024.RobotState;
import frc2024.RobotContainer.Subsystems;
import frc2024.constants.PivotConstants;
import frc2024.constants.ShooterConstants;
import frc2024.subsystems.Drivetrain;
import frc2024.subsystems.Pivot;
import frc2024.subsystems.Shooter;

public class ShootSimNote extends Command {
    
  private final Drivetrain drivetrain;
  private final Shooter shooter;
  private final Pivot pivot;

  private Translation3d[] calculatedTrajectory;
  private double shooterVelocity;
  private Rotation2d driveYaw;

  private Timer step = new Timer();

  public ShootSimNote(Subsystems subs) {
    drivetrain = subs.drivetrain();
    shooter = subs.shooter();
    pivot = subs.pivot();
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    calculatedTrajectory = RobotContainer.getRobotState().getActiveTrajectory().get();
    shooterVelocity = shooter.getVelocity();
    driveYaw = drivetrain.getPose().getRotation().unaryMinus();
    step.restart();
  }

  // Called every time the scheduler runs while the command is scheduled.
  @Override
  public void execute() {
    Translation3d point = calculatedTrajectory[getStep()];
    Rotation2d launchAngle = Rotation2d.fromRotations(pivot.getPosition()).minus(PivotConstants.ENCODER_TO_HORIZONTAL).unaryMinus();
    SmartDashboard.putNumberArray("Note", ScreamUtil.translation3dToArray(point, new Rotation3d(driveYaw.getRadians(), launchAngle.getRadians(), 0)));
  }

  private int getStep(){
    return (int) (step.get() * (1.0 / shooterVelocity) * 1000.0);
  }

  // Called once the command ends or is interrupted.
  @Override
  public void end(boolean interrupted) {}

  // Returns true when the command should end.
  @Override
  public boolean isFinished() {
    return (getStep() + 1) > ShooterConstants.NUM_TRAJECTORY_POINTS;
  }

  public void restart(){
    calculatedTrajectory = RobotContainer.getRobotState().getActiveTrajectory().get();
    shooterVelocity = shooter.getVelocity();
    step.restart();
    this.schedule();
  }
}
