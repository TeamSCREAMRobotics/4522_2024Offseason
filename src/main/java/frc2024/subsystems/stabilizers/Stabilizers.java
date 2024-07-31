package frc2024.subsystems.stabilizers;

import java.util.function.DoubleSupplier;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import com.SCREAMLib.drivers.TalonFXSubsystem;
import com.SCREAMLib.drivers.TalonFXSubsystem.TalonFXSubsystemConstants;
import com.SCREAMLib.math.Conversions;
import com.SCREAMLib.sim.SimState;
import com.ctre.phoenix6.Utils;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj2.command.Command;
import frc2024.Robot;
import frc2024.RobotContainer;
import frc2024.constants.Constants;
import frc2024.subsystems.elevator.Elevator.ElevatorGoal;
import frc2024.subsystems.pivot.PivotConstants;
import lombok.Getter;
import lombok.Setter;

public class Stabilizers extends TalonFXSubsystem{
    
    public final DCMotorSim sim = new DCMotorSim(DCMotor.getFalcon500(1), StabilizerConstants.SUBSYSTEM_CONSTANTS.rotorToSensorRatio, 0.05859096765521);
    public final PIDController simController = new PIDController(100.0, 0.0, 0.0);
    private Notifier simNotifier = null;
    private double lastSimTime;

    public Stabilizers(TalonFXSubsystemConstants constants) {
        super(constants, ElevatorGoal.TRACKING);

        if(Robot.isSimulation()){
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
    private Goal goalgoal = Goal.IDLE;

    public Command setGoalCommand(Goal goal){
        return run(() -> setGoalgoal(goal));
    }

    public Goal lastGoal = Goal.IDLE;
    @Override
    public void periodic() {
        super.periodic();
        if(getGoalgoal() == Goal.IDLE && (atGoal() && getGoalgoal() != lastGoal)){
            stop();
        } else {
            setSetpointMotionMagicPosition(getGoalgoal().getTargetRotations().getAsDouble());
        }
        lastGoal = getGoalgoal();
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
                    Conversions.rpmToRPS(sim.getAngularVelocityRPM(), StabilizerConstants.SUBSYSTEM_CONSTANTS.rotorToSensorRatio),
                    RobotController.getBatteryVoltage()));
        });
        simNotifier.startPeriodic(Constants.SIM_PERIOD_SEC);
    }
}
