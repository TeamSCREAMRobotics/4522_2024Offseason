package frc2024.subsystems.elevator;

import java.util.function.DoubleSupplier;

import org.littletonrobotics.junction.AutoLogOutput;

import com.ctre.phoenix6.Utils;
import com.team4522.lib.drivers.TalonFXSubsystem;
import com.team4522.lib.math.Conversions;
import com.team4522.lib.sim.SimulationThread;
import com.team4522.lib.sim.SimWrappers.ElevatorSimWrapper;
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

    private ElevatorSim sim;
    private SimulationThread simThread;
    private PIDController simController;

    private DoubleSupplier simVoltage;

    public Elevator(TalonFXSubsystemConstants constants) {
        super(constants);

        if(Utils.isSimulation()){
            sim = new ElevatorSim(
                DCMotor.getKrakenX60(2), 
                ElevatorConstants.ELEVATOR_CONSTANTS.rotorToSensorRatio, 
                Units.lbsToKilograms(28), 
                Units.inchesToMeters(2.211 / 2.0),
                ElevatorConstants.MIN_HEIGHT, 
                Units.inchesToMeters(ElevatorConstants.MAX_HEIGHT),
                false, 
                0.0);
            simController = new PIDController(10.0, 0.0, 0.0);
            simThread = new SimulationThread(new ElevatorSimWrapper(sim), simVoltage, this::updateSimState, Constants.SIM_PERIOD_SEC);
        }

        setDefaultCommand(moveToGoal(Goal.TRACKING));
    }

    public enum Goal {
        HOME_INTAKE(() -> 0.0),
        INTAKE_TRAP(() -> 3.41),
        SUB(() -> 3.13),
        SUB_DEFENDED(() -> 21.75),
        AMP(() -> 19.71),
        TRAP(() -> 21.75),
        EJECT(() -> 5.43),
        TRACKING(() -> RobotContainer.getRobotState().getActiveShotParameters().get().shootState().elevatorHeight());

        @Getter
        DoubleSupplier targetRotations;

        private Goal(DoubleSupplier targetHeightInches){
            targetRotations = () -> Elevator.heightInchesToRotations(targetHeightInches.getAsDouble(), ElevatorConstants.PULLEY_CIRCUMFERENCE.getInches());
        }
    }

    @Getter @Setter @AutoLogOutput(key = "RobotState/Subsystems/Elevator/Goal")
    private Goal goal = Goal.TRACKING;

    public static double heightInchesToRotations(double inches, double circumferenceInches){
        return inches / circumferenceInches;
    }

    public static Length rotationsToLength(double position, double circumferenceInches){
        return Length.fromInches(position * circumferenceInches);
    }

    public Length getHeight(){
        return rotationsToLength(getPosition(), ElevatorConstants.PULLEY_CIRCUMFERENCE.getInches());
    }

    public Command moveToGoal(Goal goal){
        return run(() -> {
            setGoal(goal);
            if(!ElevatorConstants.updateFromTuner){
                if(getGoal() == Goal.HOME_INTAKE && atGoal()){
                    stop();
                } else if(getGoal() == Goal.TRACKING){
                    setSetpointPosition(getGoal().getTargetRotations().getAsDouble());
                } else {
                    setSetpointMotionMagicPosition(getGoal().getTargetRotations().getAsDouble());
                }
            }
        }).beforeStarting(() -> simVoltage = () -> simController.calculate(getPosition(), getGoal().getTargetRotations().getAsDouble()));
    }

    public Command runVoltage(DoubleSupplier voltage){
        return run(() -> {
            setVoltage(voltage.getAsDouble());
        }).beforeStarting(() -> simVoltage = voltage);
    }

    @Override
    public void periodic() {
        super.periodic();
    }

    public void updateSimState(SimState simState){
        setSimState(
                new SimState(
                    heightInchesToRotations(Units.metersToInches(simState.position()), ElevatorConstants.PULLEY_CIRCUMFERENCE.getInches()) * ElevatorConstants.ELEVATOR_CONSTANTS.rotorToSensorRatio,
                    Conversions.mpsToFalconRPS(simState.velocity(), ElevatorConstants.PULLEY_CIRCUMFERENCE.getMeters(), ElevatorConstants.ELEVATOR_CONSTANTS.rotorToSensorRatio),
                    simState.supplyVoltage()));
    }
}
