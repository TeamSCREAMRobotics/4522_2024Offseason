package frc2024.subsystems.pivot;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import com.ctre.phoenix6.Utils;
import com.team4522.lib.drivers.TalonFXSubsystem;
import com.team4522.lib.drivers.TalonFXSubsystemGoal;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.wpilibj2.command.Command;
import frc2024.Robot;
import frc2024.RobotContainer;
import lombok.Getter;

public class Pivot extends TalonFXSubsystem{

    public Pivot(TalonFXSubsystemConstants constants) {
        super(constants, PivotGoal.TRACKING);
    }
    
    public enum PivotGoal implements TalonFXSubsystemGoal{
        HOME_INTAKE(() -> 26.317, ControlType.MOTION_MAGIC_POSITION),
        TRAP_INTAKE(() -> 53.937, ControlType.MOTION_MAGIC_POSITION),
        SUB(() -> 51.177, ControlType.MOTION_MAGIC_POSITION),
        SUB_DEFENDED(() -> 23.357, ControlType.MOTION_MAGIC_POSITION),
        AMP(() -> 22.317, ControlType.MOTION_MAGIC_POSITION),
        TRAP(() -> -8.203, ControlType.MOTION_MAGIC_POSITION),
        EJECT(() -> 24.067, ControlType.MOTION_MAGIC_POSITION),
        TRACKING(() -> RobotContainer.getRobotState().getActiveShotParameters().get().shootState().pivotAngle().getDegrees(), ControlType.POSITION);

        @Getter
        DoubleSupplier targetRotations;

        @Getter
        ControlType controlType;

        private PivotGoal(DoubleSupplier targetAngle, ControlType controlType){
            targetRotations = () -> Rotation2d.fromDegrees(targetAngle.getAsDouble()).getRotations();
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

    @Override
    public Command applyGoal(TalonFXSubsystemGoal goal){
        Supplier<Command> command;
        command = () -> {
            if(goal == PivotGoal.HOME_INTAKE && atGoal()){
                return run(() -> stop());
            } else {
                return super.applyGoal(goal);
            }
        };
        return command.get();
    }
}
