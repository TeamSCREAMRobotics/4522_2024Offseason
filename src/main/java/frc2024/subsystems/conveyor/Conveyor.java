package frc2024.subsystems.conveyor;

import java.util.function.DoubleSupplier;

import org.littletonrobotics.junction.AutoLogOutput;

import com.team4522.lib.drivers.TalonFXSubsystem;

import edu.wpi.first.wpilibj2.command.Command;
import frc2024.RobotContainer;
import frc2024.subsystems.elevator.ElevatorConstants;
import frc2024.subsystems.elevator.Elevator.ElevatorGoal;
import lombok.Getter;
import lombok.Setter;

public class Conveyor extends TalonFXSubsystem{
    
    public Conveyor(TalonFXSubsystemConstants constants){
        super(constants, ElevatorGoal.TRACKING);
    }

    public enum Goal {
        IDLING(0.0),
        INTAKING(9.0),
        EJECTING(-11.0),
        SHOOTING(11.0),
        TRAPPING(6.0);

        @Getter
        double targetOutput;

        private Goal(double output){
            this.targetOutput = output;
        }
    }

    @Getter @Setter @AutoLogOutput(key = "RobotState/Subsystems/Conveyor/Goal")
    private Goal goalgaol = Goal.IDLING;

    public Command setGoalCommand(Goal goal){
        return run(() -> setGoalgaol(goal));
    }

    @Override
    public void periodic() {
        super.periodic();
        setVoltage(getGoalgaol().getTargetOutput());
    }
}
