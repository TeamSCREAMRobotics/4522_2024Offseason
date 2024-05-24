package frc2024.subsystems;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.mechanisms.DifferentialMechanism;
import com.team4522.lib.drivers.TalonFXSubsystem;
import com.team4522.lib.math.Conversions;
import com.team4522.lib.pid.ScreamPIDConstants;
import com.team4522.lib.util.Length;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.simulation.ElevatorSim;
import edu.wpi.first.wpilibj2.command.Command;
import frc2024.RobotContainer;
import frc2024.constants.Constants;
import frc2024.constants.ElevatorConstants;
import lombok.Getter;
import lombok.Setter;

public class Elevator extends TalonFXSubsystem{

    public Elevator(TalonFXSubystemConstants constants) {
        super(constants);
    }

    public final ElevatorSim sim = 
        new ElevatorSim(
            DCMotor.getKrakenX60(2), 
            ElevatorConstants.ELEVATOR_CONSTANTS.rotorToSensorRatio, 
            Units.lbsToKilograms(28), 
            Units.inchesToMeters(2.362 / 2.0),
            ElevatorConstants.MIN_HEIGHT, 
            Units.inchesToMeters(ElevatorConstants.MAX_HEIGHT),
            false, 
            0.0
        );
    public final PIDController simController = new PIDController(78.0 * 2.0, 0.0, 0.0);

    public enum Goal {
        HOME_INTAKE(() -> 0.0),
        INTAKE_TRAP(() -> 3.41),
        SUB(() -> 3.13),
        AMP(() -> 19.71),
        TRAP(() -> 21.75),
        EJECT(() -> 5.43),
        AUTO(() -> RobotContainer.getRobotState().getShotParameters().shootState().elevatorHeight()),
        CUSTOM(() -> RobotContainer.getSubsystems().elevator().getCustomGoal());

        @Getter
        DoubleSupplier targetRotations;

        private Goal(DoubleSupplier targetHeightInches){
            targetRotations = () -> Elevator.heightInchesToRotations(targetHeightInches.getAsDouble());
        }
    }

    @Getter @Setter
    private Goal goal = Goal.AUTO;

    @Getter @Setter
    private double customGoal;

    public static double heightInchesToRotations(double inches){
        return ((inches - ElevatorConstants.MIN_HEIGHT) / (ElevatorConstants.MAX_HEIGHT - ElevatorConstants.MIN_HEIGHT)) * (ElevatorConstants.ENCODER_MAX - ElevatorConstants.ENCODER_MIN);
    }

    public static Length rotationsToLength(double position){
        return Length.fromInches(position * (ElevatorConstants.MAX_HEIGHT - ElevatorConstants.MIN_HEIGHT) / (ElevatorConstants.ENCODER_MAX - ElevatorConstants.ENCODER_MIN));
    }

    public Length getHeight(){
        return rotationsToLength(getPosition());
    }

    public Command setGoalCommand(Goal goal){
        return run(() -> setGoal(goal));
    }

    public Command setCustomGoalCommand(DoubleSupplier heightInches){
        return run(
            () -> {
                setCustomGoal(heightInches.getAsDouble());
                setGoal(Goal.CUSTOM);
            });
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
        } else {
            double inputVoltage = simController.calculate(getPosition(), getGoal().getTargetRotations().getAsDouble());
            sim.update(Constants.SIM_PERIOD_SEC);
            sim.setInputVoltage(inputVoltage);
            setSimState(
                new SimState(
                    heightInchesToRotations(Units.metersToInches(sim.getPositionMeters())),
                    0.0, 
                    inputVoltage),
                getGoal().getTargetRotations().getAsDouble(),
                false);
        }
    }
}
