package frc2024.commands;

import com.SCREAMLib.math.Conversions;
import com.SCREAMLib.math.ScreamMath;
import com.team6328.GeomUtil;
import edu.wpi.first.math.geometry.*;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj2.command.Command;
import frc2024.RobotContainer.Subsystems;
import frc2024.RobotState;
import frc2024.constants.Constants;
import frc2024.logging.NoteVisualizer;
import frc2024.subsystems.shooter.ShooterConstants;
import org.dyn4j.dynamics.Body;
import org.dyn4j.geometry.MassType;
import org.dyn4j.geometry.Transform;
import org.dyn4j.world.World;

public class ShootSimNote extends Command {
  private static int noteIndex = -1;

  private final Subsystems subsystems;
  private final World<Body> world = new World<>();
  private final Body note = new Body();

  private double shooterVelocity;
  private Rotation2d shooterAngle;
  private Rotation2d robotAngle;
  private ChassisSpeeds robotSpeed;
  private Pose3d stagedNotePosition;
  private double startTime;

  private Pose3d activeNote;

  public ShootSimNote(Subsystems subsystems) {
    setName("ShootSimNote");
    this.subsystems = subsystems;
    note.setMass(MassType.NORMAL);
    world.addBody(note);
  }

  @Override
  public void initialize() {
    shooterVelocity = calculateShooterVelocity();
    startTime = Timer.getFPGATimestamp();
    shooterAngle = subsystems.pivot().getAngle();
    robotAngle = subsystems.drivetrain().getHeading();
    robotSpeed = subsystems.drivetrain().getFieldRelativeSpeeds();
    stagedNotePosition = NoteVisualizer.getStagedNote();
    activeNote = stagedNotePosition;
    NoteVisualizer.hasNote = false;

    initializeNotePhysics();
    noteIndex++;
    if (noteIndex > 1) {
      noteIndex = 0;
    }
  }

  @Override
  public void execute() {
    activeNote = simulateNotePosition();
    RobotState.activeNotes.set(noteIndex, activeNote);
  }

  private double calculateShooterVelocity() {
    return Conversions.rpsToMPS(
        subsystems.shooter().getVelocity() * 0.8,
        ShooterConstants.WHEEL_CIRCUMFERENCE.getMeters(),
        1.0);
  }

  private void initializeNotePhysics() {
    note.setLinearVelocity(
        shooterAngle.getCos() * shooterVelocity, shooterAngle.getSin() * shooterVelocity);

    Transform transform = new Transform();
    transform.rotate(-shooterAngle.getRadians());
    note.setTransform(transform);
  }

  private Pose3d simulateNotePosition() {
    double elapsedTime = Timer.getFPGATimestamp() - startTime;
    world.update(elapsedTime);
    updateNoteVelocity();

    Translation3d noteTranslation = calculateNoteTranslation();
    return calculateNotePose(noteTranslation);
  }

  private void updateNoteVelocity() {
    note.setLinearVelocity(
        note.getLinearVelocity().x - (robotSpeed.vxMetersPerSecond * Constants.PERIOD_SEC),
        note.getLinearVelocity().y - (Constants.GRAVITY * Constants.PERIOD_SEC));
  }

  private Translation3d calculateNoteTranslation() {
    return new Translation3d(
        note.getTransform().getTranslationX(), 0.0, -note.getTransform().getTranslationY());
  }

  private Pose3d calculateNotePose(Translation3d noteTranslation) {
    Translation3d rotatedTranslation = ScreamMath.rotatePoint(noteTranslation, robotAngle);
    Transform3d tempPose =
        stagedNotePosition.minus(new Pose3d(rotatedTranslation, new Rotation3d()));
    return GeomUtil.transform3dToPose3d(
        tempPose.plus(
            new Transform3d(
                0.0, robotSpeed.vyMetersPerSecond * Constants.PERIOD_SEC, 0.0, new Rotation3d())));
  }

  @Override
  public boolean isFinished() {
    return activeNote.getZ() < 0;
  }
}
