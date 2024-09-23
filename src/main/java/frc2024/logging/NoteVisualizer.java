package frc2024.logging;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Pose3d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import frc2024.RobotContainer;
import frc2024.constants.FieldConstants;
import frc2024.subsystems.pivot.PivotConstants;
import java.util.*;

public class NoteVisualizer {
  private static final List<Translation2d> notes =
      new ArrayList<>(Arrays.asList(FieldConstants.StagingLocations.centerlineTranslations));

  public static boolean hasNote = false;

  static {
    for (Translation2d translation : FieldConstants.StagingLocations.spikeTranslations) {
      notes.add(translation);
    }
  }

  public static Pose3d[] getActiveNotes(Pose2d pose, boolean isIntaking) {
    Translation2d[] notes = NoteVisualizer.notes.toArray(Translation2d[]::new);
    if (!hasNote) {
      for (Translation2d note : notes) {
        if (isClose(note, pose) && isIntaking) {
          NoteVisualizer.notes.remove(note);
          hasNote = true;
        }
      }
    }
    return NoteVisualizer.notes.stream()
        .map(
            translation ->
                new Pose3d(
                    translation.getX(),
                    translation.getY(),
                    Units.inchesToMeters(1.0),
                    new Rotation3d()))
        .toArray(Pose3d[]::new);
  }

  public static Pose3d getStagedNote() {
    if (hasNote) {
      Pose3d componentPose =
          ComponentConstants.getShooterPose(
              RobotContainer.getSubsystems().elevator().getMeasuredHeight().getMeters(),
              RobotContainer.getSubsystems().pivot().getAngle());
      Pose2d robotPose = RobotContainer.getSubsystems().drivetrain().getPose();
      return new Pose3d(
              robotPose.getX(),
              robotPose.getY(),
              0.0,
              new Rotation3d(0, 0, robotPose.getRotation().getRadians()))
          .plus(
              new Transform3d(
                  componentPose
                      .getTranslation()
                      .plus(
                          new Translation3d(
                              Math.hypot(0, PivotConstants.SHOOTER_DISTANCE_FROM_AXLE.getMeters()),
                              componentPose
                                  .getRotation()
                                  .minus(new Rotation3d(0, Math.PI / 2.0, 0)))),
                  componentPose.getRotation()));
    } else {
      return new Pose3d();
    }
  }

  private static boolean isClose(Translation2d notePos, Pose2d robotPos) {
    return notePos.getDistance(robotPos.getTranslation()) < 0.75;
  }

  public static void resetNotes() {
    notes.clear();
    for (Translation2d translation : FieldConstants.StagingLocations.centerlineTranslations) {
      notes.add(translation);
    }
    for (Translation2d translation : FieldConstants.StagingLocations.spikeTranslations) {
      notes.add(translation);
    }
  }
}
