package frc2024.subsystems.conveyor;

import com.SCREAMLib.drivers.TalonFXSubsystem.CanDevice;
import com.SCREAMLib.drivers.TalonFXSubsystem.TalonFXConstants;
import com.SCREAMLib.drivers.TalonFXSubsystem.TalonFXSubsystemConstants;
import com.ctre.phoenix6.signals.InvertedValue;

public class ConveyorConstants {

  public static final boolean updateFromTuner = false;

  public static final int NOTE_SENSOR_ID = 2;

  public static final TalonFXSubsystemConstants SUBSYSTEM_CONSTANTS =
      new TalonFXSubsystemConstants();

  static {
    SUBSYSTEM_CONSTANTS.name = "Conveyor";

    SUBSYSTEM_CONSTANTS.outputTelemetry = false;

    SUBSYSTEM_CONSTANTS.masterConstants =
        new TalonFXConstants(new CanDevice(10, ""), InvertedValue.CounterClockwise_Positive);
  }
}
