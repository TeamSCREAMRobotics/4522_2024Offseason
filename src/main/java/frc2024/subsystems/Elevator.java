package frc2024.subsystems;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.Utils;
import com.team4522.lib.drivers.TalonFXSubsystem;
import com.team4522.lib.math.Conversions;
import com.team4522.lib.util.Length;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj2.command.Command;
import frc2024.RobotContainer;
import frc2024.constants.Constants;
import frc2024.constants.ElevatorConstants;
import lombok.Getter;
import lombok.Setter;

public class Elevator extends TalonFXSubsystem{

    private final ElevatorSim sim;
    private final PIDController simController;
    private Notifier simNotifier = null;
    private double lastSimTime;


    public Elevator(TalonFXSubystemConstants constants) {
        super(constants);

        sim = new ElevatorSim(
            DCMotor.getKrakenX60(2), 
            ElevatorConstants.ELEVATOR_CONSTANTS.rotorToSensorRatio, 
            Units.lbsToKilograms(28), 
            Units.inchesToMeters(2.362 / 2.0),
            ElevatorConstants.MIN_HEIGHT, 
            Units.inchesToMeters(ElevatorConstants.MAX_HEIGHT),
            false, 
            0.0
        );

        simController = new PIDController(15, 0.0, 0.0);

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

    @Getter @Setter
    private Goal goal = Goal.AUTO;

    @Getter @Setter
    private double customGoal;

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
        if(!ElevatorConstants.updateFromTuner && !Utils.isSimulation()){
            if(getGoal() == Goal.HOME_INTAKE && atGoal()){
                stop();
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

            double inputVoltage = simController.calculate(getPosition(), getGoal().getTargetRotations().getAsDouble());
            sim.update(deltaTime);
            sim.setInputVoltage(inputVoltage);
            updateSimState(
                new SimState(
                    heightInchesToRotations(Units.metersToInches(sim.getPositionMeters()), ElevatorConstants.PULLEY_CIRCUMFERENCE.getInches()),
                    Conversions.mpsToFalconRPS(sim.getVelocityMetersPerSecond(), ElevatorConstants.PULLEY_CIRCUMFERENCE.getMeters(), ElevatorConstants.ELEVATOR_CONSTANTS.rotorToSensorRatio),
                    RobotController.getBatteryVoltage()),
                    getGoal().getTargetRotations().getAsDouble(),
                    false);
        });
        simNotifier.startPeriodic(Constants.SIM_PERIOD_SEC);
    }
}
