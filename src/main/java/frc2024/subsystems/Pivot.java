package frc2024.subsystems;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.Utils;
import com.team4522.lib.drivers.TalonFXSubsystem;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.simulation.SingleJointedArmSim;
import edu.wpi.first.wpilibj2.command.Command;
import frc2024.RobotContainer;
import frc2024.constants.Constants;
import frc2024.constants.ElevatorConstants;
import frc2024.constants.PivotConstants;
import lombok.Getter;
import lombok.Setter;

public class Pivot extends TalonFXSubsystem{

    public Pivot(TalonFXSubystemConstants constants) {
        super(constants);
    }

    public final DCMotorSim sim = new DCMotorSim(DCMotor.getFalcon500(1), PivotConstants.PIVOT_CONSTANTS.rotorToSensorRatio, 0.366879329 + 0.03);
    public final PIDController simController = new PIDController(10000.0, 0.0, 0.0);
    
    public enum Goal{
        HOME_INTAKE(() -> 17.1),
        INTAKE_TRAP(() -> -10.52),
        SUB(() -> -7.76),
        SUB_DEFENDED(() -> 20.06),
        AMP(() -> 21.1),
        TRAP(() -> 51.62),
        EJECT(() -> 19.35),
        AIM(() -> RobotContainer.getRobotState().getShotParameters().shootState().pivotAngle().getDegrees());

        @Getter
        DoubleSupplier targetRotations;
        
        private Goal(DoubleSupplier targetAngle){
            targetRotations = () -> Rotation2d.fromDegrees(targetAngle.getAsDouble()).getRotations();
        }
    }

    @Getter @Setter
    private Goal goal = Goal.AIM;

    public Command setGoalCommand(Goal goal){
        return run(() -> setGoal(goal));
    }

    @Override
    public void periodic() {
        super.periodic();
        if(!PivotConstants.updateFromTuner && !Utils.isSimulation()){
            if(getGoal() == Goal.HOME_INTAKE && atGoal()){
                stop();
            } else {
                setSetpointPosition(getGoal().getTargetRotations().getAsDouble());
            }
        } else{
            double inputVoltage = simController.calculate(getPosition(), getGoal().getTargetRotations().getAsDouble());
            sim.update(Constants.SIM_PERIOD_SEC);
            sim.setInputVoltage(inputVoltage);
            setSimState(
                new SimState(
                    sim.getAngularPositionRotations(),
                    0.0,
                    inputVoltage),
                getGoal().getTargetRotations().getAsDouble(),
                false);
        }
    }
}
