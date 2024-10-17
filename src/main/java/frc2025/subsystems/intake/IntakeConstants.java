package frc2025.subsystems.intake;

import com.SCREAMLib.drivers.TalonFXSubsystem.CANDevice;
import com.SCREAMLib.drivers.TalonFXSubsystem.TalonFXConstants;
import com.SCREAMLib.drivers.TalonFXSubsystem.TalonFXSubsystemConstants;
import com.ctre.phoenix6.signals.InvertedValue;

public class IntakeConstants {

  public static final TalonFXSubsystemConstants SUBSYSTEM_CONSTANTS =
      new TalonFXSubsystemConstants();

  static {
    SUBSYSTEM_CONSTANTS.name = "Intake";

    SUBSYSTEM_CONSTANTS.codeEnabled = true;
    SUBSYSTEM_CONSTANTS.logTelemetry = false;

    SUBSYSTEM_CONSTANTS.masterConstants =
        new TalonFXConstants(new CANDevice(9, ""), InvertedValue.Clockwise_Positive);

    SUBSYSTEM_CONSTANTS.sensorToMechRatio = 3.0;
  }
}
