package frc2024.subsystems.stabilizers;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import org.littletonrobotics.junction.AutoLogOutput;
import org.littletonrobotics.junction.Logger;

import com.SCREAMLib.drivers.TalonFXSubsystem;
import com.SCREAMLib.drivers.TalonFXSubsystemGoal;
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
import frc2024.subsystems.pivot.Pivot.PivotGoal;
import frc2024.subsystems.pivot.PivotConstants;
import lombok.Getter;
import lombok.Setter;

public class Stabilizers extends TalonFXSubsystem{

    public Stabilizers(TalonFXSubsystemConstants constants) {
        super(constants, StabilizerGoal.IDLE);
    }

    public enum StabilizerGoal implements TalonFXSubsystemGoal{
        IDLE(() -> 0.0, ControlType.MOTION_MAGIC_POSITION),
        OUT(() -> StabilizerConstants.MAX_ANGLE.getRotations(), ControlType.MOTION_MAGIC_POSITION);

        @Getter
        DoubleSupplier targetRotations;

        @Getter
        ControlType controlType;
        
        private StabilizerGoal(DoubleSupplier targetRotations, ControlType controlType){
            this.targetRotations = targetRotations;
            this.controlType = controlType;
        }

        @Override
        public DoubleSupplier target() {
            return targetRotations;
        }

        @Override
        public ControlType controlType() {
            return controlType;
        }
    }
}
