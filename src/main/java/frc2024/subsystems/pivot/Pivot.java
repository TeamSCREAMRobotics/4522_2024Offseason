package frc2024.subsystems.pivot;

import com.SCREAMLib.drivers.TalonFXSubsystem;
import edu.wpi.first.math.geometry.Rotation2d;
import frc2024.Robot;
import frc2024.RobotContainer;
import frc2024.RobotState;
import frc2024.logging.Logger;
import java.util.function.DoubleSupplier;
import lombok.Getter;

public class Pivot extends TalonFXSubsystem {

  public Pivot(TalonFXSubsystemConstants constants) {
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
                ? HOME_INTAKE.getTarget().getAsDouble()
                : RobotState.getActiveShotParameters()
                    .get()
                    .shootState()
                    .getPivotAngle()
                    .getDegrees(),
        ControlType.POSITION);

    @Getter DoubleSupplier target;

    @Getter ControlType controlType;

    @Getter Rotation2d angle;

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

  @Override
  public void periodic() {
    super.periodic();
    Logger.log("RobotState/Subsystems/Pivot/Angle", getAngle().getDegrees());
  }
}
