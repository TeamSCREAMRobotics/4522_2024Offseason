package frc2024.subsystems.elevator;

import com.SCREAMLib.data.Length;
import com.SCREAMLib.drivers.TalonFXSubsystem.CANDevice;
import com.SCREAMLib.drivers.TalonFXSubsystem.TalonFXConstants;
import com.SCREAMLib.drivers.TalonFXSubsystem.TalonFXSubsystemConstants;
import com.SCREAMLib.drivers.TalonFXSubsystem.TalonFXSubsystemSimConstants;
import com.SCREAMLib.pid.ScreamPIDConstants;
import com.SCREAMLib.pid.ScreamPIDConstants.FeedforwardConstants;
import com.SCREAMLib.sim.SimWrapper;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;

public final class ElevatorConstants {

  public static final double MAX_HEIGHT = 21.745;
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

  public static final TalonFXSubsystemConstants SUBSYSTEM_CONSTANTS =
      new TalonFXSubsystemConstants();

  static {
    SUBSYSTEM_CONSTANTS.name = "Elevator";

    SUBSYSTEM_CONSTANTS.codeEnabled = true;
    SUBSYSTEM_CONSTANTS.outputTelemetry = false;

    SUBSYSTEM_CONSTANTS.simConstants =
        new TalonFXSubsystemSimConstants(new SimWrapper(SIM), SIM_GAINS.getPIDController());

    SUBSYSTEM_CONSTANTS.masterConstants =
        new TalonFXConstants(new CANDevice(15, ""), InvertedValue.CounterClockwise_Positive);
    SUBSYSTEM_CONSTANTS.slaveConstants =
        new TalonFXConstants[] {
          new TalonFXConstants(new CANDevice(16, ""), InvertedValue.Clockwise_Positive)
        };

    SUBSYSTEM_CONSTANTS.neutralMode = NeutralModeValue.Brake;
    SUBSYSTEM_CONSTANTS.sensorToMechRatio = GEAR_RATIO;
    SUBSYSTEM_CONSTANTS.enableSupplyCurrentLimit = true;
    SUBSYSTEM_CONSTANTS.supplyCurrentLimit = 40;
    SUBSYSTEM_CONSTANTS.minUnitsLimit = 0.0;
    SUBSYSTEM_CONSTANTS.maxUnitsLimit = 3.1;
    SUBSYSTEM_CONSTANTS.cruiseVelocity = 20.0; // 235.0
    SUBSYSTEM_CONSTANTS.acceleration = 20.0; // 30.0
    SUBSYSTEM_CONSTANTS.slot0 =
        new ScreamPIDConstants(50.0, 0, 0) // 78.0
            .getSlot0Configs(
                new FeedforwardConstants(0, 0, 0.3, 0, GravityTypeValue.Elevator_Static));
    SUBSYSTEM_CONSTANTS.positionThreshold = 0.5;
  }
}
