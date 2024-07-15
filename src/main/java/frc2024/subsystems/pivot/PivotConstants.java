package frc2024.subsystems.pivot;

import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.team4522.lib.drivers.TalonFXSubsystem.CanDevice;
import com.team4522.lib.drivers.TalonFXSubsystem.TalonFXSubsystemConstants;
import com.team4522.lib.drivers.TalonFXSubsystem.TalonFXConstants;
import com.team4522.lib.pid.ScreamPIDConstants;
import com.team4522.lib.pid.ScreamPIDConstants.FeedforwardConstants;
import com.team4522.lib.util.Length;

import edu.wpi.first.math.geometry.Rotation2d;

public final class PivotConstants {

    public static final boolean updateFromTuner = false;

    public static final TalonFXSubsystemConstants PIVOT_CONSTANTS = new TalonFXSubsystemConstants();
    static{
        PIVOT_CONSTANTS.name = "Pivot";

        PIVOT_CONSTANTS.outputTelemetry = false;

        PIVOT_CONSTANTS.masterConstants = new TalonFXConstants(new CanDevice(17, ""), InvertedValue.CounterClockwise_Positive);
        PIVOT_CONSTANTS.neutralMode = NeutralModeValue.Brake;
        
        PIVOT_CONSTANTS.rotorToSensorRatio = 125.0 * (72.0 / 22.0);
        PIVOT_CONSTANTS.sensorToMechRatio = 1.0;
        PIVOT_CONSTANTS.feedbackSensorSource = FeedbackSensorSourceValue.FusedCANcoder;
        PIVOT_CONSTANTS.feedbackRemoteSensorId = 4;
        PIVOT_CONSTANTS.feedbackRemoteSensorOffset = 0.2470703125;
        PIVOT_CONSTANTS.enableSupplyCurrentLimit = true;
        PIVOT_CONSTANTS.supplyCurrentLimit = 40;
        PIVOT_CONSTANTS.slot0 = new ScreamPIDConstants(400.0, 0, 0).getSlot0Configs(new FeedforwardConstants());
        PIVOT_CONSTANTS.positionThreshold = Rotation2d.fromDegrees(0.8).getRotations();
    }

    public static final Rotation2d MAP_OFFSET = Rotation2d.fromDegrees(-1);

    public static final Length AXLE_HEIGHT_HOME = Length.fromInches(16.640069);
    public static final Length AXLE_HEIGHT_TOP = Length.fromInches(38.059638);
    public static final Length AXLE_DISTANCE_FROM_ELEVATOR_TOP = Length.fromInches(9.975600);
    public static final Length AXLE_DISTANCE_FROM_LENS_HOME = Length.fromInches(9.189772);
    public static final Length AXLE_DISTANCE_FROM_LENS_TOP = Length.fromInches(12.966620);
    public static final Length AXLE_DISTANCE_FROM_ROBOT_CENTER_HOME = Length.fromInches(2.840333);
    public static final Length AXLE_DISTANCE_FROM_ROBOT_CENTER_TOP = Length.fromInches(6.617781);
    public static final Length SHOOTER_DISTANCE_FROM_AXLE = Length.fromInches(2.877975);
}
