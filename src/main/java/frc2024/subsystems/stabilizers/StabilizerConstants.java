package frc2024.subsystems.stabilizers;

import com.ctre.phoenix6.configs.Slot0Configs;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.team4522.lib.drivers.TalonFXSubsystem.CanDevice;
import com.team4522.lib.drivers.TalonFXSubsystem.TalonFXConstants;
import com.team4522.lib.drivers.TalonFXSubsystem.TalonFXSubsystemConstants;
import com.team4522.lib.pid.ScreamPIDConstants;
import com.team4522.lib.pid.ScreamPIDConstants.FeedforwardConstants;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.util.Units;

public class StabilizerConstants {
    
    public static final Rotation2d MIN_ANGLE = new Rotation2d();
    public static final Rotation2d MAX_ANGLE = Rotation2d.fromDegrees(128.144763);

    public static final TalonFXSubsystemConstants SUBSYSTEM_CONSTANTS = new TalonFXSubsystemConstants();
    static{
        SUBSYSTEM_CONSTANTS.name = "Stabilizers";

        SUBSYSTEM_CONSTANTS.codeEnabled = true;
        SUBSYSTEM_CONSTANTS.outputTelemetry = false;

        SUBSYSTEM_CONSTANTS.masterConstants = new TalonFXConstants(new CanDevice(13, ""), InvertedValue.CounterClockwise_Positive);
        SUBSYSTEM_CONSTANTS.neutralMode = NeutralModeValue.Brake;
        
        SUBSYSTEM_CONSTANTS.rotorToSensorRatio = 16.0;
        SUBSYSTEM_CONSTANTS.enableSupplyCurrentLimit = true;
        SUBSYSTEM_CONSTANTS.supplyCurrentLimit = 10;
        SUBSYSTEM_CONSTANTS.minUnitsLimit = 0.0;
        SUBSYSTEM_CONSTANTS.maxUnitsLimit = MAX_ANGLE.getRotations();
        SUBSYSTEM_CONSTANTS.cruiseVelocity = 15;
        SUBSYSTEM_CONSTANTS.acceleration = 5;
        SUBSYSTEM_CONSTANTS.slot0 = new ScreamPIDConstants(10, 0, 0).getSlot0Configs(new FeedforwardConstants());

        SUBSYSTEM_CONSTANTS.positionThreshold = Units.degreesToRotations(5);
    }
}
