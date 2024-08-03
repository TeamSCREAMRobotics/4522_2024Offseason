package frc2024.subsystems.elevator;

import com.SCREAMLib.data.Length;
import com.SCREAMLib.drivers.TalonFXSubsystem.CanDevice;
import com.SCREAMLib.drivers.TalonFXSubsystem.TalonFXConstants;
import com.SCREAMLib.drivers.TalonFXSubsystem.TalonFXSubsystemConstants;
import com.SCREAMLib.pid.ScreamPIDConstants;
import com.SCREAMLib.pid.ScreamPIDConstants.FeedforwardConstants;
import com.SCREAMLib.sim.SimWrapper;
import com.ctre.phoenix6.signals.GravityTypeValue;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;

public final class ElevatorConstants{

	public static final double MAX_HEIGHT = 21.745;
  	public static final double MIN_HEIGHT = 0.0;
  	public static final Length MAX_HEIGHT_AS_LENGTH = Length.fromInches(MAX_HEIGHT);
  	public static final double ENCODER_MAX = 3.1;
  	public static final double ENCODER_MIN = 0.0;
  	public static final Length HOME_HEIGHT_FROM_FLOOR = Length.fromInches(26.476152);
  	public static final Length PULLEY_CIRCUMFERENCE = Length.fromInches(6.946136755);

    public static final double GEAR_RATIO = 14.0167;

    public static final ElevatorSim SIM = new ElevatorSim(
  	  	DCMotor.getKrakenX60(2), 
  	  	GEAR_RATIO, 
  	  	Units.lbsToKilograms(31), 
  	  	Units.inchesToMeters(2.211 / 2.0),
  	  	ElevatorConstants.MIN_HEIGHT, 
  		Units.inchesToMeters(ElevatorConstants.MAX_HEIGHT),
  	  	false, 
  	  	0.0);
  	public static final ScreamPIDConstants SIM_GAINS = new ScreamPIDConstants(10.0, 0.0, 0.0);

  	public static final TalonFXSubsystemConstants SUBSYSTEM_CONSTANTS = new TalonFXSubsystemConstants();
  	static{
  	  	SUBSYSTEM_CONSTANTS.name = "Elevator";

  	  	SUBSYSTEM_CONSTANTS.codeEnabled = true;
  	  	SUBSYSTEM_CONSTANTS.outputTelemetry = true;

		SUBSYSTEM_CONSTANTS.sim = new SimWrapper(SIM);
        SUBSYSTEM_CONSTANTS.simController = SIM_GAINS.getPIDController();
        SUBSYSTEM_CONSTANTS.limitSimVoltage = false;

  	  	SUBSYSTEM_CONSTANTS.masterConstants = new TalonFXConstants(new CanDevice(15, ""), InvertedValue.CounterClockwise_Positive);
  	  	SUBSYSTEM_CONSTANTS.slaveConstants = 
  	  	  	new TalonFXConstants[]{
				new TalonFXConstants(new CanDevice(16, ""), InvertedValue.CounterClockwise_Positive)
  	  	  	};
		
  	  	SUBSYSTEM_CONSTANTS.neutralMode = NeutralModeValue.Brake;
  	  	SUBSYSTEM_CONSTANTS.rotorToSensorRatio = GEAR_RATIO;
  	  	SUBSYSTEM_CONSTANTS.enableSupplyCurrentLimit = true;
  	  	SUBSYSTEM_CONSTANTS.supplyCurrentLimit = 40;
  	  	SUBSYSTEM_CONSTANTS.minUnitsLimit = 0.0;
  	  	SUBSYSTEM_CONSTANTS.maxUnitsLimit = 3.1;
  	  	SUBSYSTEM_CONSTANTS.cruiseVelocity = 235.0;
  	  	SUBSYSTEM_CONSTANTS.acceleration = 30;
  	  	SUBSYSTEM_CONSTANTS.slot0 = new ScreamPIDConstants(78.0, 0, 0).getSlot0Configs(new FeedforwardConstants(0, 0, 0.3, 0, GravityTypeValue.Elevator_Static)); 
  	  	SUBSYSTEM_CONSTANTS.positionThreshold = 0.05 / PULLEY_CIRCUMFERENCE.getInches();
    }
}