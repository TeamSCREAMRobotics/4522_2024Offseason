package frc2025.subsystems.pivot;

import dashboard.Ligament;
import data.Length;
import drivers.TalonFXSubsystem;
import edu.wpi.first.math.geometry.Rotation2d;
import frc2025.RobotContainer;
import frc2025.RobotState;
import frc2025.logging.Logger;
import java.util.function.DoubleSupplier;

public class Pivot extends TalonFXSubsystem {

  private final Ligament pivotFront =
      new Ligament()
          .withStaticLength(Length.fromInches(17))
          .withDynamicAngle(
              () -> getAngle().unaryMinus(),
              () -> Rotation2d.fromRotations(-getGoal().target().getAsDouble()))
          .withOverrideAppend(true);

  private final Ligament pivotBack =
      new Ligament()
          .withStaticLength(Length.fromInches(9))
          .withDynamicAngle(
              () -> getAngle().unaryMinus().plus(Rotation2d.fromRotations(0.5)),
              () -> Rotation2d.fromRotations(0.5 - getGoal().target().getAsDouble()))
          .withOverrideAppend(true);

  public Pivot(TalonFXSubsystemConfiguration constants) {
    super(constants, PivotGoal.TRACKING);
  }

  public enum PivotGoal implements TalonFXSubsystemGoal {
    ZERO(() -> 0.0, ControlType.MOTION_MAGIC_POSITION),
    HOME_INTAKE(() -> 26.317, ControlType.MOTION_MAGIC_POSITION),
    TRAP_INTAKE(() -> 53.937, ControlType.MOTION_MAGIC_POSITION),
    SUB(() -> 51.177, ControlType.MOTION_MAGIC_POSITION),
    SUB_DEFENDED(() -> 23.357, ControlType.MOTION_MAGIC_POSITION),
    AMP(() -> 22.317, ControlType.MOTION_MAGIC_POSITION),
    TRAP(() -> -8.203, ControlType.MOTION_MAGIC_POSITION),
    EJECT(() -> 24.067, ControlType.MOTION_MAGIC_POSITION),
    FEED_TO_WING(() -> 51.0, ControlType.MOTION_MAGIC_POSITION),
    FEED_TO_CENTER(() -> 26.0, ControlType.MOTION_MAGIC_POSITION),
    TRACKING(
        () ->
            RobotContainer.getRobotState() == null
                ? HOME_INTAKE.target.getAsDouble()
                : RobotState.getActiveShotParameters().shootState().getPivotAngle().getDegrees(),
        ControlType.POSITION);

    public final DoubleSupplier target;

    public final ControlType controlType;

    public final Rotation2d angle;

    private PivotGoal(DoubleSupplier targetAngle, ControlType controlType) {
      this.angle = Rotation2d.fromDegrees(targetAngle.getAsDouble());
      this.target = () -> Rotation2d.fromDegrees(targetAngle.getAsDouble()).getRotations();
      this.controlType = controlType;
    }

    @Override
    public DoubleSupplier target() {
      return target;
    }

    @Override
    public ControlType controlType() {
      return controlType;
    }
  }

  public Ligament[] getLigaments() {
    return new Ligament[] {pivotFront, pivotBack};
  }

  @Override
  public void periodic() {
    super.periodic();
    Logger.log(logPrefix + "Angle", getAngle().getDegrees());
  }
}
