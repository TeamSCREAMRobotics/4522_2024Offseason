package frc2025.subsystems.intake;

import drivers.TalonFXSubsystem;
import java.util.function.DoubleSupplier;

public class Intake extends TalonFXSubsystem {

  public Intake(TalonFXSubsystemConfiguration constants) {
    super(constants, IntakeGoal.IDLE);
  }

  public enum IntakeGoal implements TalonFXSubsystemGoal {
    IDLE(0.0, ControlType.VOLTAGE),
    INTAKE(9.5, ControlType.VOLTAGE),
    EJECT(-6.0, ControlType.VOLTAGE);

    public final double voltage;
    public final ControlType controlType;

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

    @Override
    public DoubleSupplier feedForward() {
      return () -> 0.0;
    }
  }
}
