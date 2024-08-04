package frc2024.subsystems.stabilizers;

import com.SCREAMLib.drivers.TalonFXSubsystem;
import com.SCREAMLib.drivers.TalonFXSubsystemGoal;
import java.util.function.DoubleSupplier;
import lombok.Getter;

public class Stabilizers extends TalonFXSubsystem {

  public Stabilizers(TalonFXSubsystemConstants constants) {
    super(constants, StabilizerGoal.IDLE);
  }

  public enum StabilizerGoal implements TalonFXSubsystemGoal {
    IDLE(() -> 0.0, ControlType.MOTION_MAGIC_POSITION),
    OUT(() -> StabilizerConstants.MAX_ANGLE.getRotations(), ControlType.MOTION_MAGIC_POSITION);

    @Getter DoubleSupplier targetRotations;

    @Getter ControlType controlType;

    private StabilizerGoal(DoubleSupplier targetRotations, ControlType controlType) {
      this.targetRotations = targetRotations;
      this.controlType = controlType;
    }

    @Override
    public DoubleSupplier target() {
      return targetRotations;
    }

    @Override
    public ControlType controlType() {
      return controlType;
    }
  }
}
