package frc2025.subsystems.elevator;

import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import data.Length;
import drivers.TalonFXSubsystem.CANDevice;
import drivers.TalonFXSubsystem.TalonFXConstants;
import drivers.TalonFXSubsystem.TalonFXSubsystemConfiguration;
import drivers.TalonFXSubsystem.TalonFXSubsystemSimConstants;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import pid.ScreamPIDConstants;
import pid.ScreamPIDConstants.FeedforwardConstants;
import sim.SimWrapper;

public final class ElevatorConstants {

  public static final double MAX_HEIGHT = 21.513;
  public static final double MIN_HEIGHT = 0.0;
  public static final Length MAX_HEIGHT_AS_LENGTH = Length.fromInches(MAX_HEIGHT);
  public static final double ENCODER_MAX = 3.1;
  public static final double ENCODER_MIN = 0.0;
  public static final Length HOME_HEIGHT_FROM_FLOOR = Length.fromInches(26.476152);
  public static final Length PULLEY_CIRCUMFERENCE = Length.fromInches(6.946136755);

  public static final double GEAR_RATIO = 14.0167;

  public static final ElevatorSim SIM =
      new ElevatorSim(
          DCMotor.getKrakenX60(2),
          GEAR_RATIO,
          Units.lbsToKilograms(31),
          Units.inchesToMeters(2.211 / 2.0),
          MIN_HEIGHT,
          MAX_HEIGHT_AS_LENGTH.getMeters(),
          true,
          0.0);
  public static final ScreamPIDConstants SIM_GAINS = new ScreamPIDConstants(10.0, 0.0, 0.0);

  public static final TalonFXSubsystemConfiguration CONFIGURATION =
      new TalonFXSubsystemConfiguration();

  static {
    CONFIGURATION.name = "Elevator";

    CONFIGURATION.codeEnabled = true;
    CONFIGURATION.logTelemetry = false;

    CONFIGURATION.simConstants =
        new TalonFXSubsystemSimConstants(
            new SimWrapper(SIM, PULLEY_CIRCUMFERENCE, GEAR_RATIO),
            GEAR_RATIO,
            SIM_GAINS.getPIDController());

    CONFIGURATION.masterConstants =
        new TalonFXConstants(new CANDevice(15, ""), InvertedValue.Clockwise_Positive);
    CONFIGURATION.slaveConstants =
        new TalonFXConstants[] {
          new TalonFXConstants(new CANDevice(16, ""), InvertedValue.CounterClockwise_Positive)
        };

    CONFIGURATION.neutralMode = NeutralModeValue.Brake;
    CONFIGURATION.sensorToMechRatio = GEAR_RATIO;
    CONFIGURATION.enableSupplyCurrentLimit = true;
    CONFIGURATION.supplyCurrentLimit = 40;
    CONFIGURATION.minUnitsLimit = 0.0;
    CONFIGURATION.maxUnitsLimit = 3.1;
    CONFIGURATION.cruiseVelocity = 20.0;
    CONFIGURATION.acceleration = CONFIGURATION.cruiseVelocity / 0.8;
    CONFIGURATION.slot0 =
        new ScreamPIDConstants(50.0, 0, 0) // 78.0
            .getSlot0Configs(
                new FeedforwardConstants(0, 0, 0.3, 0, GravityTypeValue.Elevator_Static));
    CONFIGURATION.positionThreshold = 0.5;
  }
}
