package frc2024.subsystems.elevator;

import java.util.function.DoubleSupplier;

import org.littletonrobotics.junction.AutoLogOutput;

import com.ctre.phoenix6.Utils;
import com.team4522.lib.drivers.TalonFXSubsystem;
import com.team4522.lib.math.Conversions;
import com.team4522.lib.util.Length;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.trajectory.TrapezoidProfile.State;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj2.command.Command;
import frc2024.RobotContainer;
import frc2024.constants.Constants;
import lombok.Getter;
import lombok.Setter;

public class Elevator extends TalonFXSubsystem{

    private final ElevatorSim sim;
    private final PIDController simController;
    private Notifier simNotifier = null;
    private double lastSimTime;

    public Elevator(TalonFXSubsystemConstants constants) {
        super(constants);

        sim = new ElevatorSim(
            DCMotor.getKrakenX60(2), 
            ElevatorConstants.ELEVATOR_CONSTANTS.rotorToSensorRatio, 
            Units.lbsToKilograms(28), 
            Units.inchesToMeters(2.211 / 2.0),
            ElevatorConstants.MIN_HEIGHT, 
            Units.inchesToMeters(ElevatorConstants.MAX_HEIGHT),
            false, 
            0.0
        );

        simController = new PIDController(10.0, 0.0, 0.0);

        if(Utils.isSimulation()){
            startSimThread();
        }
    }

    public enum Goal {
        HOME_INTAKE(() -> 0.0),
        INTAKE_TRAP(() -> 3.41),
        SUB(() -> 3.13),
        AMP(() -> 19.71),
        TRAP(() -> 21.75),
        EJECT(() -> 5.43),
        AUTO(() -> RobotContainer.getRobotState().getActiveShotParameters().get().shootState().elevatorHeight());

        @Getter
        DoubleSupplier targetRotations;

        private Goal(DoubleSupplier targetHeightInches){
            targetRotations = () -> Elevator.heightInchesToRotations(targetHeightInches.getAsDouble(), ElevatorConstants.PULLEY_CIRCUMFERENCE.getInches());
        }
    }

    @Getter @Setter @AutoLogOutput(key = "RobotState/Subsystems/Elevator/Goal")
    private Goal goal = Goal.AUTO;

    public static double heightInchesToRotations(double inches, double circumferenceInches){
        return inches / circumferenceInches;
    }

    public static Length rotationsToLength(double position, double circumferenceInches){
        return Length.fromInches(position * circumferenceInches);
    }

    public Length getHeight(){
        return rotationsToLength(getPosition(), ElevatorConstants.PULLEY_CIRCUMFERENCE.getInches());
    }

    public Command setGoalCommand(Goal goal){
        return run(() -> setGoal(goal));
    }

    @Override
    public void periodic() {
        super.periodic();
        if(!ElevatorConstants.updateFromTuner){
            if(getGoal() == Goal.HOME_INTAKE && atGoal()){
                stop();
            } else if(getGoal() == Goal.AUTO){
                setSetpointPosition(getGoal().getTargetRotations().getAsDouble());
            } else {
                setSetpointMotionMagicPosition(getGoal().getTargetRotations().getAsDouble());
            }
        }
    }

    public void startSimThread(){
        simNotifier = new Notifier(() -> {
            final double currentTime = Utils.getCurrentTimeSeconds();
            double deltaTime = currentTime - lastSimTime;
            lastSimTime = currentTime;

            double inputVoltage = simController.calculate(getPosition(), getSetpoint());
            sim.update(deltaTime);
            sim.setInputVoltage(inputVoltage);
            updateSimState(
                new SimState(
                    heightInchesToRotations(Units.metersToInches(sim.getPositionMeters()), ElevatorConstants.PULLEY_CIRCUMFERENCE.getInches()) * ElevatorConstants.ELEVATOR_CONSTANTS.rotorToSensorRatio,
                    Conversions.mpsToFalconRPS(sim.getVelocityMetersPerSecond(), ElevatorConstants.PULLEY_CIRCUMFERENCE.getMeters(), ElevatorConstants.ELEVATOR_CONSTANTS.rotorToSensorRatio),
                    RobotController.getBatteryVoltage()));
        });
        simNotifier.startPeriodic(Constants.SIM_PERIOD_SEC);
    }
}
