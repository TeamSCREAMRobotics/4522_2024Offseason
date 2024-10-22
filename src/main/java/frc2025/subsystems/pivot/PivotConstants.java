package frc2025.subsystems.pivot;

import com.ctre.phoenix6.configs.CANcoderConfiguration;
import com.ctre.phoenix6.signals.AbsoluteSensorRangeValue;
import com.ctre.phoenix6.signals.FeedbackSensorSourceValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.signals.SensorDirectionValue;
import data.Length;
import drivers.TalonFXSubsystem.CANCoderConstants;
import drivers.TalonFXSubsystem.CANDevice;
import drivers.TalonFXSubsystem.TalonFXConstants;
import drivers.TalonFXSubsystem.TalonFXSubsystemConfiguration;
import drivers.TalonFXSubsystem.TalonFXSubsystemSimConstants;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import pid.ScreamPIDConstants;
import pid.ScreamPIDConstants.FeedforwardConstants;
import sim.SimWrapper;
import util.SimUtil;

public final class PivotConstants {

  public static final Rotation2d MAP_OFFSET = Rotation2d.fromDegrees(0);

  public static final Length AXLE_DISTANCE_FROM_ELEVATOR_TOP = Length.fromInches(9.998565);
  public static final Length SHOOTER_DISTANCE_FROM_AXLE = Length.fromInches(2.736100);

  public static final double GEAR_RATIO = 125.0 * (72.0 / 22.0);

  public static final DCMotorSim SIM =
      SimUtil.createDCMotorSim(DCMotor.getFalcon500(1), GEAR_RATIO, 0.6193);
  public static final ScreamPIDConstants SIM_GAINS =
      new ScreamPIDConstants(350.0 * GEAR_RATIO, 0.0, 0.0);

  public static final TalonFXSubsystemConfiguration CONFIGURATION =
      new TalonFXSubsystemConfiguration();

  static {
    CONFIGURATION.name = "Pivot";

    CONFIGURATION.codeEnabled = true;
    CONFIGURATION.logTelemetry = false;

    CONFIGURATION.simConstants =
        new TalonFXSubsystemSimConstants(
            new SimWrapper(SIM), SIM_GAINS.getPIDController(), false, false, true);

    CONFIGURATION.masterConstants =
        new TalonFXConstants(new CANDevice(17, ""), InvertedValue.Clockwise_Positive);

    CANcoderConfiguration config = new CANcoderConfiguration();
    config.MagnetSensor.AbsoluteSensorRange = AbsoluteSensorRangeValue.Signed_PlusMinusHalf;
    config.MagnetSensor.MagnetOffset = -0.12646484375;
    config.MagnetSensor.SensorDirection = SensorDirectionValue.Clockwise_Positive;
    CONFIGURATION.cancoderConstants = new CANCoderConstants(new CANDevice(4, ""), config);

    CONFIGURATION.neutralMode = NeutralModeValue.Coast;
    CONFIGURATION.rotorToSensorRatio = GEAR_RATIO;
    CONFIGURATION.feedbackSensorSource = FeedbackSensorSourceValue.FusedCANcoder;
    CONFIGURATION.feedbackRemoteSensorId = 4;
    CONFIGURATION.enableSupplyCurrentLimit = true;
    CONFIGURATION.supplyCurrentLimit = 40;
    CONFIGURATION.cruiseVelocity = 20.0;
    CONFIGURATION.acceleration = 10.0;
    CONFIGURATION.slot0 =
        new ScreamPIDConstants(350.0, 0, 0).getSlot0Configs(new FeedforwardConstants());
    CONFIGURATION.positionThreshold = 0.025;
    CONFIGURATION.minUnitsLimit = 0.0;
    CONFIGURATION.maxUnitsLimit = Units.degreesToRotations(54);
    ;
  }
}
