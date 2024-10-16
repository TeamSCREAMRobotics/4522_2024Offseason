package frc2025.subsystems.shooter;

import com.SCREAMLib.data.Length;
import com.SCREAMLib.drivers.TalonFXSubsystem.CANDevice;
import com.SCREAMLib.drivers.TalonFXSubsystem.TalonFXConstants;
import com.SCREAMLib.drivers.TalonFXSubsystem.TalonFXSubsystemConstants;
import com.SCREAMLib.drivers.TalonFXSubsystem.TalonFXSubsystemSimConstants;
import com.SCREAMLib.pid.ScreamPIDConstants;
import com.SCREAMLib.pid.ScreamPIDConstants.FeedforwardConstants;
import com.SCREAMLib.sim.SimWrapper;
import com.SCREAMLib.util.SimUtil;
import com.ctre.phoenix6.signals.InvertedValue;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import frc2025.subsystems.elevator.Elevator.ElevatorGoal;
import frc2025.subsystems.elevator.ElevatorConstants;
import frc2025.subsystems.pivot.Pivot.PivotGoal;
import frc2025.util.ShootStateInterpolatingTreeMap;
import frc2025.util.ShootStateInterpolatingTreeMap.ShootState;

public class ShooterConstants {

  public static final ScreamPIDConstants PID_CONSTANTS = new ScreamPIDConstants(0.15, 0.0, 0.0);

  public static final double KS = 0.14173;
  public static final double KV = 0.11;
  public static final double KA = 0.0;
  public static final double KG = 0.0;
  public static final FeedforwardConstants FEEDFORWARD_CONSTANTS =
      new FeedforwardConstants(KV, KS, KG, KA);

  public static final Length WHEEL_CIRCUMFERENCE = Length.fromInches(4.0 * Math.PI);

  public static final Length SHOOTER_BACK_LENGTH = Length.fromInches(9.0);
  public static final Length SHOOTER_FRONT_LENGTH = Length.fromInches(15.5);

  public static final FlywheelSim SIM =
      SimUtil.createFlywheelSim(DCMotor.getFalcon500(1), 1.0, 0.00599676919909 + 0.0001);
  public static final ScreamPIDConstants SIM_GAINS = new ScreamPIDConstants(0.001, 0, 0);
  public static final SimpleMotorFeedforward SIM_FEEDFORWARD =
      new SimpleMotorFeedforward(0.08902, 0.11053, 0.00045);

  public static final TalonFXSubsystemConstants SUBSYSTEM_CONSTANTS =
      new TalonFXSubsystemConstants();

  static {
    SUBSYSTEM_CONSTANTS.name = "Shooter";

    SUBSYSTEM_CONSTANTS.codeEnabled = true;
    SUBSYSTEM_CONSTANTS.logTelemetry = true;

    SUBSYSTEM_CONSTANTS.simConstants =
        new TalonFXSubsystemSimConstants(new SimWrapper(SIM), SIM_GAINS.getPIDController());

    SUBSYSTEM_CONSTANTS.masterConstants =
        new TalonFXConstants(new CANDevice(11, ""), InvertedValue.CounterClockwise_Positive);
    SUBSYSTEM_CONSTANTS.slaveConstants =
        new TalonFXConstants[] {
          new TalonFXConstants(new CANDevice(12, ""), InvertedValue.CounterClockwise_Positive)
        };
    SUBSYSTEM_CONSTANTS.slot0 =
        new ScreamPIDConstants(0.15, 0, 0).getSlot0Configs(FEEDFORWARD_CONSTANTS);

    SUBSYSTEM_CONSTANTS.velocityThreshold = 1.25;
  }

  public static final ShootStateInterpolatingTreeMap SHOOTING_MAP =
      new ShootStateInterpolatingTreeMap();

  static {
    SHOOTING_MAP.put(8.0, new ShootState(Rotation2d.fromDegrees(19.50), 0.0, 4500.0));
    SHOOTING_MAP.put(7.5, new ShootState(Rotation2d.fromDegrees(19.75), 0.0, 4500.0));
    SHOOTING_MAP.put(7.0, new ShootState(Rotation2d.fromDegrees(20.0), 0.0, 4500.0));
    SHOOTING_MAP.put(6.5, new ShootState(Rotation2d.fromDegrees(21.3), 0.0, 4250.0));
    SHOOTING_MAP.put(6.0, new ShootState(Rotation2d.fromDegrees(21.8), 0.0, 4250.0));
    SHOOTING_MAP.put(5.5, new ShootState(Rotation2d.fromDegrees(22.3), 0.0, 4250.0));
    SHOOTING_MAP.put(5.0, new ShootState(Rotation2d.fromDegrees(24.15), 0.0, 4000.0));
    SHOOTING_MAP.put(4.5, new ShootState(Rotation2d.fromDegrees(28.3 - 3.5), 0.0, 4000.0));
    SHOOTING_MAP.put(4.0, new ShootState(Rotation2d.fromDegrees(30.5 - 3.65), 0.0, 4000.0));
    SHOOTING_MAP.put(3.5, new ShootState(Rotation2d.fromDegrees(33.3 - 4.0), 0.0, 4000.0));
    SHOOTING_MAP.put(3.0, new ShootState(Rotation2d.fromDegrees(37.0 - 5.0), 0.0, 4000.0));
    SHOOTING_MAP.put(2.5, new ShootState(Rotation2d.fromDegrees(41.3 - 5.0), 0.0, 4000.0));
    SHOOTING_MAP.put(2.0, new ShootState(Rotation2d.fromDegrees(47.4 - 6.0), 0.0, 3500.0));
    SHOOTING_MAP.put(
        1.5,
        new ShootState(
            Rotation2d.fromDegrees(55.0 - 6.0),
            Length.fromRotations(
                    ElevatorGoal.SUB.getTarget().getAsDouble(),
                    ElevatorConstants.PULLEY_CIRCUMFERENCE)
                .getInches(),
            3250.0));
    SHOOTING_MAP.put(
        1.0,
        new ShootState(
            Rotation2d.fromRotations(PivotGoal.SUB.getTarget().getAsDouble()),
            Length.fromRotations(
                    ElevatorGoal.SUB.getTarget().getAsDouble(),
                    ElevatorConstants.PULLEY_CIRCUMFERENCE)
                .getInches(),
            3000.0));
  }
}
