package frc2025.subsystems.conveyor;

import com.ctre.phoenix6.signals.InvertedValue;
import drivers.TalonFXSubsystem.CANDevice;
import drivers.TalonFXSubsystem.TalonFXConstants;
import drivers.TalonFXSubsystem.TalonFXSubsystemConfiguration;

public class ConveyorConstants {

  public static final boolean updateFromTuner = false;

  public static final int NOTE_SENSOR_ID = 2;

  public static final TalonFXSubsystemConfiguration CONFIGURATION =
      new TalonFXSubsystemConfiguration();

  static {
    CONFIGURATION.name = "Conveyor";

    CONFIGURATION.codeEnabled = true;
    CONFIGURATION.logTelemetry = false;

    CONFIGURATION.masterConstants =
        new TalonFXConstants(new CANDevice(10, ""), InvertedValue.CounterClockwise_Positive);

    CONFIGURATION.sensorToMechRatio = 4.0;
  }
}
