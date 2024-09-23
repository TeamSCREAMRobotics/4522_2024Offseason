// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc2024.commands;

import com.SCREAMLib.math.Conversions;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc2024.RobotContainer.Subsystems;
import frc2024.RobotState;
import frc2024.logging.NoteVisualizer;
import frc2024.subsystems.shooter.ShooterConstants;

public class ShootSimNote extends Command {
  private final Subsystems subsystems;

  private Translation3d[] calculatedTrajectory;
  private double shooterVelocity;
  private double shooterAngle;
  private double robotAngle;

  private double startTime;
  private double duration;

  public ShootSimNote(Subsystems subsystems) {
    this.subsystems = subsystems;
  }

  // Called when the command is initially scheduled.
  @Override
  public void initialize() {
    calculatedTrajectory = RobotState.getActiveTrajectory().get();
    shooterVelocity = subsystems.shooter().getVelocity();
    startTime = Timer.getFPGATimestamp();
    duration =
        calculatedTrajectory[0].getDistance(calculatedTrajectory[calculatedTrajectory.length - 1])
            / Conversions.rpsToMPS(
                shooterVelocity, ShooterConstants.WHEEL_CIRCUMFERENCE.getMeters(), 1.0);
    shooterAngle = subsystems.pivot().getAngle().getRadians();
    robotAngle = subsystems.drivetrain().getHeading().getRadians();

    RobotState.addActiveNote(
        () -> new Pose3d(getNoteTranslation(), new Rotation3d(0, shooterAngle, robotAngle)));
  }

  private Translation3d getNoteTranslation() {
    double totalDuration = duration * calculatedTrajectory.length;
    double normalizedTime = (double) (Timer.getFPGATimestamp() - startTime) / totalDuration;

    int currentSegment = (int) Math.floor(normalizedTime * (calculatedTrajectory.length - 1));
    currentSegment = Math.max(0, Math.min(currentSegment, calculatedTrajectory.length - 2));

    double segmentDuration = 1.0 / (calculatedTrajectory.length - 1);
    double segmentTime = (normalizedTime - currentSegment * segmentDuration) / segmentDuration;

    Translation3d t1 = calculatedTrajectory[currentSegment];
    Translation3d t2 = calculatedTrajectory[currentSegment + 1];

    return t1.interpolate(t2, segmentTime);
  }

  @Override
  public boolean isFinished() {
    return true;
  }

  @Override
  public void end(boolean interrupted) {
    NoteVisualizer.hasNote = false;
  }
}
