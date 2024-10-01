package frc2024.logging;

import com.SCREAMLib.util.AllianceFlipUtil;
import com.SCREAMLib.util.ScreamUtil;
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
                new Pose3d(translation.getX(), translation.getY(), NOTE_HEIGHT, new Rotation3d()))
        .toArray(Pose3d[]::new);
  }

  public static Pose3d getStagedNote() {
    if (!hasNote) {
      return ScreamUtil.EMPTY_POSE3D;
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

  private static boolean isClose(Translation2d notePos, Pose2d robotPos) {
    return notePos.getDistance(robotPos.getTranslation()) < INTAKE_DISTANCE;
  }

  public static void resetNotes() {
    notes.clear();
    notes.addAll(Arrays.asList(FieldConstants.StagingLocations.centerlineTranslations));
    notes.addAll(Arrays.asList(FieldConstants.StagingLocations.spikeTranslations));
    notes.add(AllianceFlipUtil.MirroredTranslation2d(new Translation2d(15.3, 1.0)));
    hasNote = true;
  }
}
