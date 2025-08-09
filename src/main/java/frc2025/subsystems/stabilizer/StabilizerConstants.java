package frc2025.subsystems.stabilizer;

import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import drivers.TalonFXSubsystem.CANDevice;
import drivers.TalonFXSubsystem.TalonFXConstants;
import drivers.TalonFXSubsystem.TalonFXSubsystemConfiguration;
import drivers.TalonFXSubsystem.TalonFXSubsystemSimConstants;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import pid.ScreamPIDConstants;
import pid.ScreamPIDConstants.FeedforwardConstants;
import sim.SimWrapper;
import util.SimUtil;

public class StabilizerConstants {

  public static final Rotation2d MIN_ANGLE = Rotation2d.kZero;
  public static final Rotation2d MAX_ANGLE = Rotation2d.fromRotations(0.099);

  public static final double REAL_POS_TO_SIM_SCALAR = 3.5842;

  public static final double GEAR_RATIO = 16.0;

  public static final DCMotorSim SIM =
      SimUtil.createDCMotorSim(DCMotor.getFalcon500(1), GEAR_RATIO, 0.05859096765521);
  public static final ScreamPIDConstants SIM_GAINS = new ScreamPIDConstants(75.0, 0.0, 0.0);

  public static final TalonFXSubsystemConfiguration CONFIGURATION =
      new TalonFXSubsystemConfiguration();

  static {
    CONFIGURATION.name = "Stabilizer";

    CONFIGURATION.codeEnabled = true;
    CONFIGURATION.logTelemetry = false;

    CONFIGURATION.simConstants =
        new TalonFXSubsystemSimConstants(
            new SimWrapper(SIM),
            GEAR_RATIO,
            SIM_GAINS.getProfiledPIDController(new Constraints(99999999, 99999999)),
            false,
            false);

    CONFIGURATION.masterConstants =
        new TalonFXConstants(new CANDevice(13, ""), InvertedValue.CounterClockwise_Positive);
    CONFIGURATION.neutralMode = NeutralModeValue.Brake;

    CONFIGURATION.sensorToMechRatio = GEAR_RATIO;
    CONFIGURATION.enableSupplyCurrentLimit = true;
    CONFIGURATION.supplyCurrentLimit = 10;
    CONFIGURATION.minUnitsLimit = 0.0;
    CONFIGURATION.maxUnitsLimit = MAX_ANGLE.getRotations();
    CONFIGURATION.cruiseVelocity = 15;
    CONFIGURATION.acceleration = 5;
    CONFIGURATION.slot0 =
        new ScreamPIDConstants(10, 0, 0).getSlot0Configs(new FeedforwardConstants());

    CONFIGURATION.positionThreshold = Units.degreesToRotations(5);
  }
}
