package frc2024.subsystems.shooter;

import com.ctre.phoenix6.signals.InvertedValue;
import com.team4522.lib.drivers.TalonFXSubsystem.CanDevice;
import com.team4522.lib.drivers.TalonFXSubsystem.TalonFXConstants;
import com.team4522.lib.drivers.TalonFXSubsystem.TalonFXSubsystemConstants;
import com.team4522.lib.pid.ScreamPIDConstants;
import com.team4522.lib.pid.ScreamPIDConstants.FeedforwardConstants;
import com.team4522.lib.util.Length;
import com.team4522.lib.util.ShootStateInterpolatingTreeMap;

import edu.wpi.first.math.geometry.Rotation2d;
import frc2024.subsystems.elevator.Elevator;
import frc2024.subsystems.elevator.ElevatorConstants;
import frc2024.subsystems.pivot.Pivot;
import frc2024.subsystems.pivot.PivotConstants;
import frc2024.subsystems.shooter.ShootingUtils.ShootState;

public class ShooterConstants {

    public static final boolean updateFromTuner = false;

    public static final TalonFXSubsystemConstants SHOOTER_CONSTANTS = new TalonFXSubsystemConstants();
    static{
        SHOOTER_CONSTANTS.name = "Shooter";

        SHOOTER_CONSTANTS.outputTelemetry = true;

        SHOOTER_CONSTANTS.masterConstants = new TalonFXConstants(new CanDevice(11, ""), InvertedValue.CounterClockwise_Positive);
        SHOOTER_CONSTANTS.slaveConstants = 
            new TalonFXConstants[]{
                new TalonFXConstants(new CanDevice(12, ""), InvertedValue.Clockwise_Positive)
            };

        SHOOTER_CONSTANTS.velocityThreshold = 1.25;
    }

    public static final ScreamPIDConstants PID_CONSTANTS = new ScreamPIDConstants(0.15, 0.0, 0.0);

    public static final double KS = 0.14173;
    public static final double KV = 0.11;
    public static final double KA = 0.0;
    public static final double KG = 0.0;
    public static final FeedforwardConstants FEEDFORWARD_CONSTANTS = new FeedforwardConstants(KV, KS, KG, KA);

    public static final Length WHEEL_CIRCUMFERENCE = Length.fromInches(4.0 * Math.PI);

    public static final int NUM_TRAJECTORY_POINTS = 30;

    public static final ShootStateInterpolatingTreeMap SHOOTING_MAP = new ShootStateInterpolatingTreeMap();
    static{
        SHOOTING_MAP.put(8.0, new ShootState(Rotation2d.fromDegrees(20.15), 0.0, 4000.0));
        SHOOTING_MAP.put(7.5, new ShootState(Rotation2d.fromDegrees(20.0), 0.0, 4000.0));
        SHOOTING_MAP.put(7.0, new ShootState(Rotation2d.fromDegrees(20.5), 0.0, 4000.0));
        SHOOTING_MAP.put(6.5, new ShootState(Rotation2d.fromDegrees(20.9), 0.0, 4000.0));
        SHOOTING_MAP.put(6.0, new ShootState(Rotation2d.fromDegrees(21.5), 0.0, 4000.0));
        SHOOTING_MAP.put(5.5, new ShootState(Rotation2d.fromDegrees(22.5), 0.0, 4000.0));
        SHOOTING_MAP.put(5.0, new ShootState(Rotation2d.fromDegrees(23.1), 0.0, 4000.0));
        SHOOTING_MAP.put(4.5, new ShootState(Rotation2d.fromDegrees(28.3 - 3.5), 0.0, 4000.0));
        SHOOTING_MAP.put(4.0, new ShootState(Rotation2d.fromDegrees(30.5 - 3.5), 0.0, 4000.0));
        SHOOTING_MAP.put(3.5, new ShootState(Rotation2d.fromDegrees(33.3 - 4.0), 0.0, 4000.0));
        SHOOTING_MAP.put(3.0, new ShootState(Rotation2d.fromDegrees(37.0 - 5.0), 0.0, 4000.0));
        SHOOTING_MAP.put(2.5, new ShootState(Rotation2d.fromDegrees(41.3 - 5.0), 0.0, 4000.0));
        SHOOTING_MAP.put(2.0, new ShootState(Rotation2d.fromDegrees(47.4 - 6.0), Elevator.rotationsToLength(Elevator.Goal.SUB.getTargetRotations().getAsDouble(), ElevatorConstants.PULLEY_CIRCUMFERENCE.getInches()).getInches(), 3500.0));
        SHOOTING_MAP.put(1.5, new ShootState(Rotation2d.fromDegrees(55.0 - 6.0), Elevator.rotationsToLength(Elevator.Goal.SUB.getTargetRotations().getAsDouble(), ElevatorConstants.PULLEY_CIRCUMFERENCE.getInches()).getInches(), 3250.0));
        SHOOTING_MAP.put(1.0, new ShootState(Rotation2d.fromRotations(Pivot.Goal.SUB.getTargetRotations().getAsDouble()), Elevator.rotationsToLength(Elevator.Goal.SUB.getTargetRotations().getAsDouble(), ElevatorConstants.PULLEY_CIRCUMFERENCE.getInches()).getInches(), 3000.0));
    }
}
