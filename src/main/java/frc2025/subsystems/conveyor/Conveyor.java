package frc2025.subsystems.conveyor;

import drivers.TalonFXSubsystem;
import edu.wpi.first.wpilibj.DigitalInput;
import frc2025.Robot;
import frc2025.logging.NoteVisualizer;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import lombok.Getter;

public class Conveyor extends TalonFXSubsystem {

  DigitalInput noteSensor;

  public Conveyor(TalonFXSubsystemConfiguration constants) {
    super(constants, ConveyorGoal.IDLE);
    noteSensor = new DigitalInput(ConveyorConstants.NOTE_SENSOR_ID);
  }

  public enum ConveyorGoal implements TalonFXSubsystemGoal {
    IDLE(0.0, ControlType.VOLTAGE),
    INTAKE(9.0, ControlType.VOLTAGE),
    EJECT(-11.0, ControlType.VOLTAGE),
    SHOOT(11.0, ControlType.VOLTAGE),
    TRAP(6.0, ControlType.VOLTAGE);

    @Getter double target;

    @Getter ControlType controlType;

    private ConveyorGoal(double voltage, ControlType controlType) {
      this.target = voltage;
      this.controlType = controlType;
    }

    @Override
    public DoubleSupplier target() {
      return () -> target;
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

  public BooleanSupplier hasNote() {
    return () -> !noteSensor.get() || (Robot.isSimulation() && NoteVisualizer.hasNote);
  }
}
