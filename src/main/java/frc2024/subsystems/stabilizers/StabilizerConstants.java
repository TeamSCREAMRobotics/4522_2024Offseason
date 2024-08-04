package frc2024.subsystems.stabilizers;

import com.SCREAMLib.drivers.TalonFXSubsystem.CanDevice;
import com.SCREAMLib.drivers.TalonFXSubsystem.TalonFXConstants;
import com.SCREAMLib.drivers.TalonFXSubsystem.TalonFXSubsystemConstants;
import com.SCREAMLib.pid.ScreamPIDConstants;
import com.SCREAMLib.pid.ScreamPIDConstants.FeedforwardConstants;
import com.SCREAMLib.sim.SimWrapper;
import com.ctre.phoenix6.signals.InvertedValue;
import com.ctre.phoenix6.signals.NeutralModeValue;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;

public class StabilizerConstants {

  public static final Rotation2d MIN_ANGLE = new Rotation2d();
  public static final Rotation2d MAX_ANGLE = Rotation2d.fromDegrees(128.144763);

  public static final double GEAR_RATIO = 16.0;

  public static final DCMotorSim SIM =
      new DCMotorSim(DCMotor.getFalcon500(1), GEAR_RATIO, 0.05859096765521);
  public static final ScreamPIDConstants SIM_GAINS = new ScreamPIDConstants(100.0, 0.0, 0.0);

  public static final TalonFXSubsystemConstants SUBSYSTEM_CONSTANTS =
      new TalonFXSubsystemConstants();

  static {
    SUBSYSTEM_CONSTANTS.name = "Stabilizers";

    SUBSYSTEM_CONSTANTS.codeEnabled = true;
    SUBSYSTEM_CONSTANTS.outputTelemetry = false;

    SUBSYSTEM_CONSTANTS.sim = new SimWrapper(SIM);
    SUBSYSTEM_CONSTANTS.simController = SIM_GAINS.getPIDController();
    SUBSYSTEM_CONSTANTS.limitSimVoltage = true;

    SUBSYSTEM_CONSTANTS.masterConstants =
        new TalonFXConstants(new CanDevice(13, ""), InvertedValue.CounterClockwise_Positive);
    SUBSYSTEM_CONSTANTS.neutralMode = NeutralModeValue.Brake;

    SUBSYSTEM_CONSTANTS.rotorToSensorRatio = GEAR_RATIO;
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
