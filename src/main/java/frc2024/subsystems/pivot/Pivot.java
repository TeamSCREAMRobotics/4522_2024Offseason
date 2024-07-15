package frc2024.subsystems.pivot;

import java.util.function.DoubleSupplier;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedDashboardNumber;

import com.ctre.phoenix6.Utils;
import com.team4522.lib.drivers.TalonFXSubsystem;
import com.team4522.lib.math.Conversions;

import edu.wpi.first.math.controller.ArmFeedforward;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj2.command.Command;
import frc2024.RobotContainer;
import frc2024.constants.Constants;
import frc2024.subsystems.elevator.ElevatorConstants;
import lombok.Getter;
import lombok.Setter;

public class Pivot extends TalonFXSubsystem{

    public final DCMotorSim sim = new DCMotorSim(DCMotor.getFalcon500(1), PivotConstants.PIVOT_CONSTANTS.rotorToSensorRatio, 0.366879329 + 0.03);
    public final PIDController simController = new PIDController(150000.0, 0.0, 0.0);
    private Notifier simNotifier = null;
    private double lastSimTime;

    public Pivot(TalonFXSubsystemConstants constants) {
        super(constants);

        if(Utils.isSimulation()){
            startSimThread();
        }
    }
    
    public enum Goal{
        HOME_INTAKE(() -> 26.317),
        INTAKE_TRAP(() -> 53.937),
        SUB(() -> 51.177),
        SUB_DEFENDED(() -> 23.357),
        AMP(() -> 22.317),
        TRAP(() -> -8.203),
        EJECT(() -> 24.067),
        AIM(() -> RobotContainer.getRobotState().getActiveShotParameters().get().shootState().pivotAngle().getDegrees());

        @Getter
        DoubleSupplier targetRotations;
        
        private Goal(DoubleSupplier targetAngle){
            targetRotations = () -> Rotation2d.fromDegrees(targetAngle.getAsDouble()).getRotations();
        }
    }

    @Getter @Setter @AutoLogOutput(key = "RobotState/Subsystems/Pivot/Goal")
    private Goal goal = Goal.AIM;

    public Command setGoalCommand(Goal goal){
        return run(() -> setGoal(goal));
    }

    @Override
    public void periodic() {
        super.periodic();
        if(!PivotConstants.updateFromTuner){
            if(getGoal() == Goal.HOME_INTAKE && atGoal()){
                stop();
            } else if(getGoal() == Goal.AIM){
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
                    sim.getAngularPositionRotations(),
                    Conversions.rpmToFalconRPS(sim.getAngularVelocityRPM(), PivotConstants.PIVOT_CONSTANTS.rotorToSensorRatio),
                    RobotController.getBatteryVoltage()));
        });
        simNotifier.startPeriodic(Constants.SIM_PERIOD_SEC);
    }
}
