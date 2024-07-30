package frc2024.subsystems.pivot;

import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.team4522.lib.data.Length;
import com.team4522.lib.drivers.TalonFXSubsystem.CanDevice;
import com.team4522.lib.drivers.TalonFXSubsystem.TalonFXSubsystemConstants;
import com.team4522.lib.drivers.TalonFXSubsystem.TalonFXConstants;
import com.team4522.lib.pid.ScreamPIDConstants;
import com.team4522.lib.pid.ScreamPIDConstants.FeedforwardConstants;
import com.team4522.lib.sim.SimWrapper;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

public final class PivotConstants {

    public static final Rotation2d MAP_OFFSET = Rotation2d.fromDegrees(0);

    public static final Length AXLE_DISTANCE_FROM_ELEVATOR_TOP = Length.fromInches(9.998565);
    public static final Length SHOOTER_DISTANCE_FROM_AXLE = Length.fromInches(2.736100);

    public static final double GEAR_RATIO = 125.0 * (72.0 / 22.0);

    public static final DCMotorSim SIM = new DCMotorSim(DCMotor.getFalcon500(1), GEAR_RATIO, 0.366879329 + 0.03);
    public static final ScreamPIDConstants SIM_GAINS = new ScreamPIDConstants(150000.0, 0.0, 0.0);

    public static final TalonFXSubsystemConstants SUBSYSTEM_CONSTANTS = new TalonFXSubsystemConstants();
    static{
        SUBSYSTEM_CONSTANTS.name = "Pivot";

        SUBSYSTEM_CONSTANTS.codeEnabled = true;
        SUBSYSTEM_CONSTANTS.outputTelemetry = false;

        SUBSYSTEM_CONSTANTS.sim = new SimWrapper(SIM);
        SUBSYSTEM_CONSTANTS.simController = SIM_GAINS.getPIDController();
        SUBSYSTEM_CONSTANTS.limitSimVoltage = false;

        SUBSYSTEM_CONSTANTS.masterConstants = new TalonFXConstants(new CanDevice(17, ""), InvertedValue.CounterClockwise_Positive);
        SUBSYSTEM_CONSTANTS.neutralMode = NeutralModeValue.Brake;
        
        SUBSYSTEM_CONSTANTS.rotorToSensorRatio = GEAR_RATIO;
        SUBSYSTEM_CONSTANTS.sensorToMechRatio = 1.0;
        SUBSYSTEM_CONSTANTS.feedbackSensorSource = FeedbackSensorSourceValue.FusedCANcoder;
        SUBSYSTEM_CONSTANTS.feedbackRemoteSensorId = 4;
        SUBSYSTEM_CONSTANTS.feedbackRemoteSensorOffset = 0.2470703125;
        SUBSYSTEM_CONSTANTS.enableSupplyCurrentLimit = true;
        SUBSYSTEM_CONSTANTS.supplyCurrentLimit = 40;
        SUBSYSTEM_CONSTANTS.slot0 = new ScreamPIDConstants(400.0, 0, 0).getSlot0Configs(new FeedforwardConstants());
        SUBSYSTEM_CONSTANTS.positionThreshold = Units.degreesToRotations(2.0);
    }
}
