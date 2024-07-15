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

    public static final boolean updateFromTuner = false;
    
    public static final Rotation2d MIN_ANGLE = new Rotation2d();
    public static final Rotation2d MAX_ANGLE = Rotation2d.fromDegrees(128.144763);

    public static final TalonFXSubsystemConstants STABILIZER_CONSTANTS = new TalonFXSubsystemConstants();
    static{
        STABILIZER_CONSTANTS.name = "Stabilizers";

        STABILIZER_CONSTANTS.outputTelemetry = false;

        STABILIZER_CONSTANTS.masterConstants = new TalonFXConstants(new CanDevice(13, ""), InvertedValue.CounterClockwise_Positive);
        STABILIZER_CONSTANTS.neutralMode = NeutralModeValue.Brake;
        
        STABILIZER_CONSTANTS.rotorToSensorRatio = 16.0;
        STABILIZER_CONSTANTS.enableSupplyCurrentLimit = true;
        STABILIZER_CONSTANTS.supplyCurrentLimit = 10;
        STABILIZER_CONSTANTS.minUnitsLimit = 0.0;
        STABILIZER_CONSTANTS.maxUnitsLimit = MAX_ANGLE.getRotations();
        STABILIZER_CONSTANTS.cruiseVelocity = 15;
        STABILIZER_CONSTANTS.acceleration = 5;
        STABILIZER_CONSTANTS.slot0 = new ScreamPIDConstants(10, 0, 0).getSlot0Configs(new FeedforwardConstants());

        STABILIZER_CONSTANTS.positionThreshold = Units.degreesToRotations(5);
    }
}
