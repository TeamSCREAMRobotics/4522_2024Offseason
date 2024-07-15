package frc2024.subsystems.shooter;

import java.util.function.DoubleSupplier;

import org.littletonrobotics.junction.AutoLog;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedDashboardNumber;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.configs.CustomParamsConfigs;
import com.team4522.lib.drivers.TalonFXSubsystem;
import com.team4522.lib.math.Conversions;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc2024.RobotContainer;
import frc2024.constants.Constants;
import frc2024.subsystems.pivot.PivotConstants;
import lombok.Getter;
import lombok.Setter;

public class Shooter extends TalonFXSubsystem{

    public final FlywheelSim sim = new FlywheelSim(DCMotor.getFalcon500(1), 1.0, 0.00599676919909 + 0.0001);
    public final PIDController simController = new PIDController(0.001, 0.0, 0.0);
    public final SimpleMotorFeedforward simFeedforward = new SimpleMotorFeedforward(0.08902, 0.10953, 0.0001);
    private Notifier simNotifier = null;
    private double lastSimTime;

    public Shooter(TalonFXSubsystemConstants constants) {
        super(constants);

        if(Utils.isSimulation()){
            startSimThread();
        }
    }

    public enum Goal{
        SUB(() -> 3000),
        AUTO(() -> RobotContainer.getRobotState().getActiveShotParameters().get().shootState().velocityRPM());

        @Getter
        DoubleSupplier targetRPS;
        
        private Goal(DoubleSupplier targetRPM){
            targetRPS = () -> targetRPM.getAsDouble() / 60.0;
        }
    }

    @Getter @Setter @AutoLogOutput(key = "RobotState/Subsystems/Shooter/Goal")
    private Goal goal = Goal.AUTO;

    public Command setGoalCommand(Goal goal){
        return run(() -> setGoal(goal));
    }

    @Override
    public void periodic() {
        super.periodic();
        if(!ShooterConstants.updateFromTuner){
            setSetpointVelocity(getGoal().getTargetRPS().getAsDouble());
        }
    }

    public void startSimThread(){
        simNotifier = new Notifier(() -> {
            final double currentTime = Utils.getCurrentTimeSeconds();
            double deltaTime = currentTime - lastSimTime;
            lastSimTime = currentTime;

            double inputVoltage = simFeedforward.calculate(getVelocity(), getGoal().getTargetRPS().getAsDouble(), deltaTime) + simController.calculate(getVelocity(), getGoal().getTargetRPS().getAsDouble());
            sim.update(deltaTime);
            sim.setInputVoltage(inputVoltage);
            updateSimState(
                new SimState(
                    0.0,
                    sim.getAngularVelocityRPM() / 60.0,
                    RobotController.getBatteryVoltage()));
        });
        simNotifier.startPeriodic(Constants.SIM_PERIOD_SEC);
    }
}
