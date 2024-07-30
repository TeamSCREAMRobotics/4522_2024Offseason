package frc2024.subsystems.shooter;

import java.util.function.DoubleSupplier;

import org.littletonrobotics.junction.AutoLog;
import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.LoggedDashboardNumber;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.configs.CustomParamsConfigs;
import com.team4522.lib.drivers.TalonFXSubsystem;
import com.team4522.lib.drivers.TalonFXSubsystemGoal;
import com.team4522.lib.math.Conversions;
import com.team4522.lib.sim.SimState;
import com.team4522.lib.sim.SimWrapper;
import com.team4522.lib.sim.SimulationThread;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.units.Distance;
import edu.wpi.first.units.Measure;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc2024.Robot;
import frc2024.RobotContainer;
import frc2024.constants.Constants;
import frc2024.subsystems.elevator.Elevator.ElevatorGoal;
import frc2024.subsystems.pivot.PivotConstants;
import lombok.Getter;
import lombok.Setter;

public class Shooter extends TalonFXSubsystem{

    private SimpleMotorFeedforward simff;

    public Shooter(TalonFXSubsystemConstants constants) {
        super(constants, ShooterGoal.TRACKING);

        if(Robot.isSimulation()){
            simff = ShooterConstants.SIM_FEEDFORWARD;
            simFeedforward = () -> simff.calculate(getVelocity(), getSetpoint(), constants.simPeriodSec);
        }
    }

    public enum ShooterGoal implements TalonFXSubsystemGoal{
        SUB(() -> 3000, ControlType.VELOCITY),
        TRACKING(() -> RobotContainer.getRobotState().getActiveShotParameters().get().shootState().velocityRPM(), ControlType.VELOCITY);

        @Getter
        DoubleSupplier targetRPS;

        @Getter
        ControlType controlType;
        
        private ShooterGoal(DoubleSupplier targetRPM, ControlType controlType){
            targetRPS = () -> targetRPM.getAsDouble() / 60.0;
            this.controlType = controlType;
        }

        @Override
        public DoubleSupplier target() {
            return targetRPS;
        }

        @Override
        public ControlType controlType() {
            return controlType;
        }
    }
}
