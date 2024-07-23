package frc2024.subsystems.stabilizers;

import java.util.function.DoubleSupplier;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import com.ctre.phoenix6.Utils;
import com.team4522.lib.drivers.TalonFXSubsystem;
import com.team4522.lib.math.Conversions;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj2.command.Command;
import frc2024.RobotContainer;
import frc2024.constants.Constants;
import frc2024.subsystems.pivot.PivotConstants;
import lombok.Getter;
import lombok.Setter;

public class Stabilizers extends TalonFXSubsystem{
    
    public final DCMotorSim sim = new DCMotorSim(DCMotor.getFalcon500(1), StabilizerConstants.STABILIZER_CONSTANTS.rotorToSensorRatio, 0.05859096765521);
    public final PIDController simController = new PIDController(100.0, 0.0, 0.0);
    private Notifier simNotifier = null;
    private double lastSimTime;

    public Stabilizers(TalonFXSubsystemConstants constants) {
        super(constants);

        if(Utils.isSimulation()){
            startSimThread();
        }

        setDefaultCommand(setGoalCommand(Goal.IDLE));
    }

    public enum Goal{
        IDLE(() -> 0.0),
        OUT(() -> StabilizerConstants.MAX_ANGLE.getDegrees());

        @Getter
        DoubleSupplier targetRotations;
        
        private Goal(DoubleSupplier targetAngle){
            targetRotations = () -> Rotation2d.fromDegrees(targetAngle.getAsDouble()).getRotations();
        }
    }

    @Getter @Setter @AutoLogOutput(key = "RobotState/Subsystems/Stabilizers/Goal")
    private Goal goal = Goal.IDLE;

    public Command setGoalCommand(Goal goal){
        return run(() -> setGoal(goal));
    }

    public Goal lastGoal = Goal.IDLE;
    @Override
    public void periodic() {
        super.periodic();
        if(!StabilizerConstants.updateFromTuner){
            if(getGoal() == Goal.IDLE && (atGoal() && getGoal() != lastGoal)){
                stop();
            } else {
                setSetpointMotionMagicPosition(getGoal().getTargetRotations().getAsDouble());
            }
        }
        lastGoal = getGoal();
    }

    public void startSimThread(){
        simNotifier = new Notifier(() -> {
            final double currentTime = Utils.getCurrentTimeSeconds();
            double deltaTime = currentTime - lastSimTime;
            lastSimTime = currentTime;

            double inputVoltage = simController.calculate(getPosition(), getSetpoint());
            sim.update(deltaTime);
            sim.setInputVoltage(inputVoltage);
            setSimState(
                new SimState(
                    sim.getAngularPositionRotations(),
                    Conversions.rpmToFalconRPS(sim.getAngularVelocityRPM(), StabilizerConstants.STABILIZER_CONSTANTS.rotorToSensorRatio),
                    RobotController.getBatteryVoltage()));
        });
        simNotifier.startPeriodic(Constants.SIM_PERIOD_SEC);
    }
}
