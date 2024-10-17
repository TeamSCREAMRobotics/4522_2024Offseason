package frc2025.subsystems.stabilizer;

import com.SCREAMLib.drivers.TalonFXSubsystem.CANDevice;
import com.SCREAMLib.drivers.TalonFXSubsystem.TalonFXConstants;
import com.SCREAMLib.drivers.TalonFXSubsystem.TalonFXSubsystemConstants;
import com.SCREAMLib.drivers.TalonFXSubsystem.TalonFXSubsystemSimConstants;
import com.SCREAMLib.pid.ScreamPIDConstants;
import com.SCREAMLib.pid.ScreamPIDConstants.FeedforwardConstants;
import com.SCREAMLib.sim.SimWrapper;
import com.SCREAMLib.util.SimUtil;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

public class StabilizerConstants {

  public static final Rotation2d MIN_ANGLE = Rotation2d.kZero;
  public static final Rotation2d MAX_ANGLE = Rotation2d.fromRotations(0.099);

  public static final double REAL_POS_TO_SIM_SCALAR = 3.5842;

  public static final double GEAR_RATIO = 16.0;

  public static final DCMotorSim SIM =
      SimUtil.createDCMotorSim(DCMotor.getFalcon500(1), GEAR_RATIO, 0.05859096765521);
  public static final ScreamPIDConstants SIM_GAINS = new ScreamPIDConstants(75.0, 0.0, 0.0);

  public static final TalonFXSubsystemConstants SUBSYSTEM_CONSTANTS =
      new TalonFXSubsystemConstants();

  static {
    SUBSYSTEM_CONSTANTS.name = "Stabilizer";

    SUBSYSTEM_CONSTANTS.codeEnabled = true;
    SUBSYSTEM_CONSTANTS.logTelemetry = false;

    SUBSYSTEM_CONSTANTS.simConstants =
        new TalonFXSubsystemSimConstants(
            new SimWrapper(SIM), SIM_GAINS.getPIDController(), false, false, false);

    SUBSYSTEM_CONSTANTS.masterConstants =
        new TalonFXConstants(new CANDevice(13, ""), InvertedValue.CounterClockwise_Positive);
    SUBSYSTEM_CONSTANTS.neutralMode = NeutralModeValue.Brake;

    SUBSYSTEM_CONSTANTS.sensorToMechRatio = GEAR_RATIO;
    SUBSYSTEM_CONSTANTS.enableSupplyCurrentLimit = true;
    SUBSYSTEM_CONSTANTS.supplyCurrentLimit = 10;
    SUBSYSTEM_CONSTANTS.minUnitsLimit = 0.0;
    SUBSYSTEM_CONSTANTS.maxUnitsLimit = MAX_ANGLE.getRotations();
    SUBSYSTEM_CONSTANTS.cruiseVelocity = 15;
    SUBSYSTEM_CONSTANTS.acceleration = 5;
    SUBSYSTEM_CONSTANTS.slot0 =
        new ScreamPIDConstants(10, 0, 0).getSlot0Configs(new FeedforwardConstants());

    SUBSYSTEM_CONSTANTS.positionThreshold = Units.degreesToRotations(5);
  }
}
