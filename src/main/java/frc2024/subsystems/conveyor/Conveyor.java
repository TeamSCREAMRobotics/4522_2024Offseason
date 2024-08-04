package frc2024.subsystems.conveyor;

import com.SCREAMLib.drivers.TalonFXSubsystem;
import com.SCREAMLib.drivers.TalonFXSubsystemGoal;
import edu.wpi.first.wpilibj.DigitalInput;
import frc2024.Robot;
import java.util.function.DoubleSupplier;
import lombok.Getter;

public class Conveyor extends TalonFXSubsystem {

  DigitalInput noteSensor;

  public Conveyor(TalonFXSubsystemConstants constants) {
    super(constants, ConveyorGoal.IDLING);
    noteSensor = new DigitalInput(ConveyorConstants.NOTE_SENSOR_ID);
  }

  public enum ConveyorGoal implements TalonFXSubsystemGoal {
    IDLING(0.0, ControlType.VOLTAGE),
    INTAKING(9.0, ControlType.VOLTAGE),
    EJECTING(-11.0, ControlType.VOLTAGE),
    SHOOTING(11.0, ControlType.VOLTAGE),
    TRAPPING(6.0, ControlType.VOLTAGE);

    @Getter double targetVoltage;

    @Getter ControlType controlType;

    private ConveyorGoal(double voltage, ControlType controlType) {
      this.targetVoltage = voltage;
      this.controlType = controlType;
    }

    @Override
    public DoubleSupplier target() {
      return () -> targetVoltage;
    }

    @Override
    public ControlType controlType() {
      return controlType;
    }
  }

  public boolean hasNote() {
    return !noteSensor.get() || Robot.isSimulation();
  }
}
