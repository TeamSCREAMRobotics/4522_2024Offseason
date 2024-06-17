package frc2024.subsystems;

import java.util.function.DoubleSupplier;

import com.ctre.phoenix6.Utils;
import com.team4522.lib.drivers.TalonFXSubsystem;
import com.team4522.lib.math.Conversions;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj2.command.Command;
import frc2024.RobotContainer;
import frc2024.constants.Constants;
import frc2024.constants.ElevatorConstants;
import frc2024.constants.PivotConstants;
import lombok.Getter;
import lombok.Setter;

public class Pivot extends TalonFXSubsystem{

    public final DCMotorSim sim = new DCMotorSim(DCMotor.getFalcon500(1), PivotConstants.PIVOT_CONSTANTS.rotorToSensorRatio, 0.366879329 + 0.03);
    public final PIDController simController = new PIDController(400.0, 0.0, 0.0);
    private Notifier simNotifier = null;
    private double lastSimTime;

    public Pivot(TalonFXSubystemConstants constants) {
        super(constants);

        if(Utils.isSimulation()){
            startSimThread();
        }
    }
    
    public enum Goal{
        HOME_INTAKE(() -> 17.1),
        INTAKE_TRAP(() -> -10.52),
        SUB(() -> -7.76),
        SUB_DEFENDED(() -> 20.06),
        AMP(() -> 21.1),
        TRAP(() -> 51.62),
        EJECT(() -> 19.35),
        AIM(() -> RobotContainer.getRobotState().getActiveShotParameters().get().shootState().pivotAngle().getDegrees());

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
                    sim.getAngularPositionRotations(),
                    Conversions.rpmToFalconRPS(sim.getAngularVelocityRPM(), PivotConstants.PIVOT_CONSTANTS.rotorToSensorRatio),
                    RobotController.getBatteryVoltage()),
                    getGoal().getTargetRotations().getAsDouble(),
                    false);
        });
        simNotifier.startPeriodic(Constants.SIM_PERIOD_SEC);
    }
}
