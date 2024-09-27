package frc2024.subsystems.intake;

import com.SCREAMLib.drivers.TalonFXSubsystem;
import java.util.function.DoubleSupplier;
import lombok.Getter;

public class Intake extends TalonFXSubsystem {

  public Intake(TalonFXSubsystemConstants constants) {
    super(constants, IntakeGoal.IDLE);
  }

  public enum IntakeGoal implements TalonFXSubsystemGoal {
    IDLE(0.0, ControlType.VOLTAGE),
    INTAKE(9.5, ControlType.VOLTAGE),
    EJECT(-6.0, ControlType.VOLTAGE);

    @Getter double voltage;
    @Getter ControlType controlType;

    private IntakeGoal(double voltage, ControlType controlType) {
      this.voltage = voltage;
      this.controlType = controlType;
    }

    @Override
    public DoubleSupplier target() {
      return () -> voltage;
    }

    @Override
    public ControlType controlType() {
      return controlType;
    }
  }
}
