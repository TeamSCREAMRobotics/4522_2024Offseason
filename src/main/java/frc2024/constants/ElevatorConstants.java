package frc2024.constants;

import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.team4522.lib.drivers.TalonFXSubsystem.CanDevice;
import com.team4522.lib.drivers.TalonFXSubsystem.TalonFXSubystemConstants;
import com.team4522.lib.drivers.TalonFXSubsystem.TalonFXConstants;
import com.team4522.lib.pid.ScreamPIDConstants;
import com.team4522.lib.pid.ScreamPIDConstants.FeedforwardConstants;
import com.team4522.lib.util.Length;

import frc2024.subsystems.Elevator;

public final class ElevatorConstants{

  public static final boolean updateFromTuner = false;

  public static final double MAX_HEIGHT = 21.75;  
  public static final double MIN_HEIGHT = 0.0;
  public static final Length MAX_HEIGHT_AS_LENGTH = Length.fromInches(21.75);
  public static final double ENCODER_MAX = 3.1;
  public static final double ENCODER_MIN = 0.0;
  public static final Length HOME_HEIGHT_FROM_FLOOR = Length.fromInches(26.553805);
  public static final Length PULLEY_CIRCUMFERENCE = Length.fromInches(2.211024 * Math.PI);

  public static final TalonFXSubystemConstants ELEVATOR_CONSTANTS = new TalonFXSubystemConstants();
  static{
    ELEVATOR_CONSTANTS.name = "Elevator";
    ELEVATOR_CONSTANTS.outputTelemetry = false;
    ELEVATOR_CONSTANTS.masterConstants = new TalonFXConstants(new CanDevice(15, ""), InvertedValue.CounterClockwise_Positive);
    ELEVATOR_CONSTANTS.slaveConstants = 
      new TalonFXConstants[]{
        new TalonFXConstants(new CanDevice(16, ""), InvertedValue.CounterClockwise_Positive)
      };
    ELEVATOR_CONSTANTS.neutralMode = NeutralModeValue.Brake;
    ELEVATOR_CONSTANTS.rotorToSensorRatio = 14.0167;
    ELEVATOR_CONSTANTS.enableSupplyCurrentLimit = true;
    ELEVATOR_CONSTANTS.supplyCurrentLimit = 40;
    ELEVATOR_CONSTANTS.minUnitsLimit = 0.0;
    ELEVATOR_CONSTANTS.maxUnitsLimit = 3.1;
    ELEVATOR_CONSTANTS.cruiseVelocity = 235.0;
    ELEVATOR_CONSTANTS.acceleration = 30;
    ELEVATOR_CONSTANTS.slot0 = new ScreamPIDConstants(78.0, 0, 0).getSlot0Configs(new FeedforwardConstants(0, 0, 0.3, 0, GravityTypeValue.Elevator_Static)); 
    ELEVATOR_CONSTANTS.positionThreshold = 0.05 / PULLEY_CIRCUMFERENCE.getInches();
  }
}