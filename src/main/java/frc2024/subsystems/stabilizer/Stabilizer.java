package frc2024.subsystems.stabilizer;

import com.SCREAMLib.drivers.TalonFXSubsystem;
import frc2024.logging.Logger;
import java.util.function.DoubleSupplier;
import lombok.Getter;

public class Stabilizer extends TalonFXSubsystem {

  public Stabilizer(TalonFXSubsystemConstants constants) {
    super(constants, StabilizerGoal.IDLE);

    resetPosition(0.0);
  }

  public enum StabilizerGoal implements TalonFXSubsystemGoal {
    IDLE(() -> 0.0, ControlType.MOTION_MAGIC_POSITION),
    OUT(() -> StabilizerConstants.MAX_ANGLE.getRotations(), ControlType.MOTION_MAGIC_POSITION);

    @Getter DoubleSupplier target;

    @Getter ControlType controlType;

    private StabilizerGoal(DoubleSupplier targetRotations, ControlType controlType) {
      this.target = targetRotations;
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
    Logger.log(logPrefix + "Angle", getAngle());
  }
}
