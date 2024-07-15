package frc2024.subsystems.conveyor;

import com.ctre.phoenix6.signals.InvertedValue;
import com.team4522.lib.drivers.TalonFXSubsystem.CanDevice;
import com.team4522.lib.drivers.TalonFXSubsystem.TalonFXConstants;
import com.team4522.lib.drivers.TalonFXSubsystem.TalonFXSubsystemConstants;

public class ConveyorConstants {

    public static final boolean updateFromTuner = false;

    public static final TalonFXSubsystemConstants CONVEYOR_CONSTANTS = new TalonFXSubsystemConstants();
    static{
        CONVEYOR_CONSTANTS.name = "Conveyor";

        CONVEYOR_CONSTANTS.outputTelemetry = false;

        CONVEYOR_CONSTANTS.masterConstants = new TalonFXConstants(new CanDevice(10, ""), InvertedValue.CounterClockwise_Positive);
    }
    
}
