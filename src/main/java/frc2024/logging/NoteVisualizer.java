package frc2024.logging;

import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.util.Units;
import frc2024.RobotContainer;
import frc2024.constants.FieldConstants;
import frc2024.subsystems.pivot.PivotConstants;
import java.util.*;

public class NoteVisualizer {
  private static final Set<Translation2d> notes = new HashSet<>();
  private static final double INTAKE_DISTANCE = 0.55;
  private static final double NOTE_HEIGHT = Units.inchesToMeters(1.0);

  public static boolean hasNote = false;

  static {
    resetNotes();
  }

  public static Pose3d[] getActiveNotes(Pose2d pose, boolean isIntaking) {
    if (!hasNote && isIntaking) {
      for (Iterator<Translation2d> iterator = notes.iterator(); iterator.hasNext(); ) {
        Translation2d note = iterator.next();
        if (isClose(note, pose)) {
          iterator.remove();
          hasNote = true;
          break;
        }
      }
    }

    return notes.stream()
        .map(
            translation ->
                new Pose3d(translation.getX(), translation.getY(), NOTE_HEIGHT, new Rotation3d()))
        .toArray(Pose3d[]::new);
  }

  public static Pose3d getStagedNote() {
    if (!hasNote) {
      return new Pose3d();
    }

    Pose2d robotPose = RobotContainer.getSubsystems().drivetrain().getPose();
    double elevatorHeight =
        RobotContainer.getSubsystems().elevator().getMeasuredHeight().getMeters();
    Rotation2d pivotAngle = RobotContainer.getSubsystems().pivot().getAngle();

    Pose3d componentPose = ComponentConstants.getShooterPose(elevatorHeight, pivotAngle);
    Translation3d noteOffset =
        new Translation3d(PivotConstants.SHOOTER_DISTANCE_FROM_AXLE.getMeters(), 0, 0)
            .rotateBy(componentPose.getRotation().minus(new Rotation3d(0, Math.PI / 2.0, 0)));

    return new Pose3d(
            robotPose.getX(),
            robotPose.getY(),
            0.0,
            new Rotation3d(0, 0, robotPose.getRotation().getRadians()))
        .plus(
            new Transform3d(
                componentPose.getTranslation().plus(noteOffset), componentPose.getRotation()));
  }

  private static boolean isClose(Translation2d notePos, Pose2d robotPos) {
    return notePos.getDistance(robotPos.getTranslation()) < INTAKE_DISTANCE;
  }

  public static void resetNotes() {
    notes.clear();
    notes.addAll(Arrays.asList(FieldConstants.StagingLocations.centerlineTranslations));
    notes.addAll(Arrays.asList(FieldConstants.StagingLocations.spikeTranslations));
    hasNote = false;
  }
}
