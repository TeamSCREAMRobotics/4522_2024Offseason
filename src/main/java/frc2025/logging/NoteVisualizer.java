package frc2025.logging;

import edu.wpi.first.math.geometry.*;
import frc2025.RobotContainer;
import frc2025.constants.FieldConstants;
import frc2025.subsystems.pivot.PivotConstants;
import java.util.*;
import util.AllianceFlipUtil;

public class NoteVisualizer {
  private static final Set<Translation2d> notes = new HashSet<>();
  private static final double INTAKE_DISTANCE = 0.55;

  public static boolean hasNote = false;

  static {
    resetNotes();
  }

  public static Pose3d[] getActiveNotes(Pose2d pose, boolean isIntaking) {
    if (!hasNote && isIntaking) {
      for (Iterator<Translation2d> iterator = notes.iterator(); iterator.hasNext(); ) {
        Translation2d note = iterator.next();
        if (isClose(note, pose)) {
          if (!note.equals(AllianceFlipUtil.MirroredTranslation2d(new Translation2d(15.3, 1.0)))) {
            iterator.remove();
          }
          hasNote = true;
          break;
        }
      }
    }

    return notes.stream()
        .map(
            translation ->
                new Pose3d(
                    translation.getX(),
                    translation.getY(),
                    FieldConstants.NOTE_HEIGHT.getMeters(),
                    Rotation3d.kZero))
        .toArray(Pose3d[]::new);
  }

  public static Pose3d getStagedNote() {
    if (!hasNote) {
      return Pose3d.kZero;
    }

    var drivetrain = RobotContainer.getSubsystems().drivetrain();
    var elevator = RobotContainer.getSubsystems().elevator();
    var pivot = RobotContainer.getSubsystems().pivot();

    Pose2d robotPose = drivetrain.getPose();

    Pose3d componentPose =
        ComponentConstants.getShooterPose(
            elevator.getMeasuredHeight().getMeters(), pivot.getAngle());
    Rotation3d componentRotation = componentPose.getRotation();

    double offsetX =
        PivotConstants.SHOOTER_DISTANCE_FROM_AXLE.getMeters() * Math.cos(componentRotation.getY());
    double offsetZ =
        PivotConstants.SHOOTER_DISTANCE_FROM_AXLE.getMeters() * Math.sin(componentRotation.getY());

    Translation3d finalTranslation =
        new Translation3d(
            robotPose.getX()
                + (componentPose.getX() + offsetX) * robotPose.getRotation().getCos()
                - (componentPose.getY()) * robotPose.getRotation().getSin(),
            robotPose.getY()
                + (componentPose.getX() + offsetX) * robotPose.getRotation().getSin()
                + (componentPose.getY()) * robotPose.getRotation().getCos(),
            componentPose.getZ() + offsetZ);

    return new Pose3d(
        finalTranslation,
        new Rotation3d(0, componentRotation.getY(), robotPose.getRotation().getRadians()));
  }

  private static boolean isClose(Translation2d notePos, Pose2d robotPose) {
    return notePos.getDistance(robotPose.getTranslation()) < INTAKE_DISTANCE;
  }

  public static Optional<Translation2d> getClosestNote(Pose2d robotPose) {
    return notes.stream()
        .min(Comparator.comparingDouble(note -> note.getDistance(robotPose.getTranslation())));
  }

  public static void resetNotes() {
    notes.clear();
    notes.addAll(Arrays.asList(FieldConstants.StagingLocations.centerlineTranslations));
    notes.addAll(Arrays.asList(FieldConstants.StagingLocations.spikeTranslations));
    notes.add(AllianceFlipUtil.MirroredTranslation2d(new Translation2d(15.3, 1.0)));
    hasNote = true;
  }
}
