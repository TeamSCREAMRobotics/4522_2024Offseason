package frc2025.subsystems.pivot;

import com.SCREAMLib.data.Length;
import com.SCREAMLib.drivers.TalonFXSubsystem.CANCoderConstants;
import com.SCREAMLib.drivers.TalonFXSubsystem.CANDevice;
import com.SCREAMLib.drivers.TalonFXSubsystem.TalonFXConstants;
import com.SCREAMLib.drivers.TalonFXSubsystem.TalonFXSubsystemConstants;
import com.SCREAMLib.drivers.TalonFXSubsystem.TalonFXSubsystemSimConstants;
import com.SCREAMLib.pid.ScreamPIDConstants;
import com.SCREAMLib.pid.ScreamPIDConstants.FeedforwardConstants;
import com.SCREAMLib.sim.SimWrapper;
import com.SCREAMLib.util.SimUtil;
import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

public final class PivotConstants {

  public static final Rotation2d MAP_OFFSET = Rotation2d.fromDegrees(0);

  public static final Length AXLE_DISTANCE_FROM_ELEVATOR_TOP = Length.fromInches(9.998565);
  public static final Length SHOOTER_DISTANCE_FROM_AXLE = Length.fromInches(2.736100);

  public static final double GEAR_RATIO = 125.0 * (72.0 / 22.0);

  public static final DCMotorSim SIM =
      SimUtil.createDCMotorSim(DCMotor.getFalcon500(1), GEAR_RATIO, 0.6193);
  public static final ScreamPIDConstants SIM_GAINS =
      new ScreamPIDConstants(350.0 * GEAR_RATIO, 0.0, 0.0);

  public static final TalonFXSubsystemConstants SUBSYSTEM_CONSTANTS =
      new TalonFXSubsystemConstants();

  static {
    SUBSYSTEM_CONSTANTS.name = "Pivot";

    SUBSYSTEM_CONSTANTS.codeEnabled = true;
    SUBSYSTEM_CONSTANTS.logTelemetry = false;

    SUBSYSTEM_CONSTANTS.simConstants =
        new TalonFXSubsystemSimConstants(
            new SimWrapper(SIM), SIM_GAINS.getPIDController(), false, false, true);

    SUBSYSTEM_CONSTANTS.masterConstants =
        new TalonFXConstants(new CANDevice(17, ""), InvertedValue.Clockwise_Positive);

    CANcoderConfiguration config = new CANcoderConfiguration();
    config.MagnetSensor.AbsoluteSensorRange = AbsoluteSensorRangeValue.Signed_PlusMinusHalf;
    config.MagnetSensor.MagnetOffset = -0.12646484375;
    config.MagnetSensor.SensorDirection = SensorDirectionValue.Clockwise_Positive;
    SUBSYSTEM_CONSTANTS.cancoderConstants = new CANCoderConstants(new CANDevice(4, ""), config);

    SUBSYSTEM_CONSTANTS.neutralMode = NeutralModeValue.Coast;
    SUBSYSTEM_CONSTANTS.rotorToSensorRatio = GEAR_RATIO;
    SUBSYSTEM_CONSTANTS.feedbackSensorSource = FeedbackSensorSourceValue.FusedCANcoder;
    SUBSYSTEM_CONSTANTS.feedbackRemoteSensorId = 4;
    SUBSYSTEM_CONSTANTS.enableSupplyCurrentLimit = true;
    SUBSYSTEM_CONSTANTS.supplyCurrentLimit = 40;
    SUBSYSTEM_CONSTANTS.cruiseVelocity = 20.0;
    SUBSYSTEM_CONSTANTS.acceleration = 10.0;
    SUBSYSTEM_CONSTANTS.slot0 =
        new ScreamPIDConstants(350.0, 0, 0).getSlot0Configs(new FeedforwardConstants());
    SUBSYSTEM_CONSTANTS.positionThreshold = 0.025;
    SUBSYSTEM_CONSTANTS.minUnitsLimit = 0.0;
    SUBSYSTEM_CONSTANTS.maxUnitsLimit = Units.degreesToRotations(54);
    ;
  }
}
