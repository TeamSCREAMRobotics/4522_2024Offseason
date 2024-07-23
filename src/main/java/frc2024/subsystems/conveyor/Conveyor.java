package frc2024.subsystems.conveyor;

import java.util.function.DoubleSupplier;

import org.littletonrobotics.junction.AutoLogOutput;

import com.team4522.lib.drivers.TalonFXSubsystem;

import edu.wpi.first.wpilibj2.command.Command;
import frc2024.RobotContainer;
import frc2024.subsystems.elevator.ElevatorConstants;
import lombok.Getter;
import lombok.Setter;

public class Conveyor extends TalonFXSubsystem{
    
    public Conveyor(TalonFXSubsystemConstants constants){
        super(constants);

        setDefaultCommand(setGoalCommand(Goal.IDLING));
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
    private Goal goal = Goal.IDLING;

    public Command setGoalCommand(Goal goal){
        return run(() -> setGoal(goal));
    }

    @Override
    public void periodic() {
        super.periodic();
        if(!ConveyorConstants.updateFromTuner){
            setVoltage(getGoal().getTargetOutput());
        }
    }
}
