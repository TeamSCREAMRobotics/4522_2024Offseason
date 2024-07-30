package frc2024.subsystems.conveyor;

import com.ctre.phoenix6.signals.InvertedValue;
import com.team4522.lib.drivers.TalonFXSubsystem.CanDevice;
import com.team4522.lib.drivers.TalonFXSubsystem.TalonFXConstants;
import com.team4522.lib.drivers.TalonFXSubsystem.TalonFXSubsystemConstants;

public class ConveyorConstants {

    public static final boolean updateFromTuner = false;

    public static final TalonFXSubsystemConstants SUBSYSTEM_CONSTANTS = new TalonFXSubsystemConstants();
    static{
        SUBSYSTEM_CONSTANTS.name = "Conveyor";

        SUBSYSTEM_CONSTANTS.outputTelemetry = false;

        SUBSYSTEM_CONSTANTS.masterConstants = new TalonFXConstants(new CanDevice(10, ""), InvertedValue.CounterClockwise_Positive);
    }
    
}
