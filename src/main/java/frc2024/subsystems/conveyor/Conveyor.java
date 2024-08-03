package frc2024.subsystems.conveyor;

import java.util.function.DoubleSupplier;

import org.littletonrobotics.junction.AutoLogOutput;

import com.SCREAMLib.drivers.TalonFXSubsystem;
import com.SCREAMLib.drivers.TalonFXSubsystemGoal;

import edu.wpi.first.wpilibj2.command.Command;
import frc2024.RobotContainer;
import frc2024.subsystems.elevator.ElevatorConstants;
import frc2024.subsystems.elevator.Elevator.ElevatorGoal;
import lombok.Getter;
import lombok.Setter;

public class Conveyor extends TalonFXSubsystem{
    
    public Conveyor(TalonFXSubsystemConstants constants){
        super(constants, ConveyorGoal.IDLING);
    }

    public enum ConveyorGoal implements TalonFXSubsystemGoal{
        IDLING(0.0, ControlType.VOLTAGE),
        INTAKING(9.0, ControlType.VOLTAGE),
        EJECTING(-11.0, ControlType.VOLTAGE),
        SHOOTING(11.0, ControlType.VOLTAGE),
        TRAPPING(6.0, ControlType.VOLTAGE);

        @Getter
        double targetVoltage;

        @Getter
        ControlType controlType;

        private ConveyorGoal(double voltage, ControlType controlType){
            this.targetVoltage = voltage;
            this.controlType = controlType;
        }

        @Override
        public DoubleSupplier target() {
            return () -> targetVoltage;
        }

        @Override
        public ControlType controlType() {
            return controlType;
        }
    }
}
