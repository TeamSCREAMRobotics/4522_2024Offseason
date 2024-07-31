package frc2024.subsystems.elevator;

import java.util.function.DoubleSupplier;

import com.SCREAMLib.data.Length;
import com.SCREAMLib.drivers.TalonFXSubsystem;
import com.SCREAMLib.drivers.TalonFXSubsystemGoal;
import com.SCREAMLib.math.Conversions;
import com.SCREAMLib.sim.SimState;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.wpilibj2.command.Command;
import frc2024.Robot;
import frc2024.RobotContainer;
import frc2024.subsystems.shooter.ShootingUtils;
import lombok.Getter;

public class Elevator extends TalonFXSubsystem{

    public Elevator(TalonFXSubsystemConstants constants) {
        super(constants, ElevatorGoal.TRACKING);
    }

    public enum ElevatorGoal implements TalonFXSubsystemGoal{
        HOME_INTAKE(() -> 0.0, ControlType.MOTION_MAGIC_POSITION),
        TRAP_INTAKE(() -> 3.41, ControlType.MOTION_MAGIC_POSITION),
        SUB(() -> 3.13, ControlType.MOTION_MAGIC_POSITION),
        SUB_DEFENDED(() -> 21.75, ControlType.MOTION_MAGIC_POSITION),
        AMP(() -> 19.71, ControlType.MOTION_MAGIC_POSITION),
        TRAP(() -> 21.75, ControlType.MOTION_MAGIC_POSITION),
        EJECT(() -> 5.43, ControlType.MOTION_MAGIC_POSITION),
        TRACKING(() -> RobotContainer.getRobotState().getActiveShotParameters().get().shootState().elevatorHeight(), ControlType.POSITION);

        @Getter
        DoubleSupplier targetRotations;

        @Getter
        ControlType controlType;

        private ElevatorGoal(DoubleSupplier targetHeightInches, ControlType controlType){
            targetRotations = () -> Conversions.linearDistanceToRotations(Length.fromInches(targetHeightInches.getAsDouble()), ElevatorConstants.PULLEY_CIRCUMFERENCE);
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

    public Length getHeight(){
        return Length.fromRotations(getPosition(), ElevatorConstants.PULLEY_CIRCUMFERENCE);
    }

    @Override
    public void setSimState(SimState simState){
        super.setSimState(
                new SimState(
                    Conversions.linearDistanceToRotations(Length.fromMeters(simState.position()), ElevatorConstants.PULLEY_CIRCUMFERENCE) * ElevatorConstants.SUBSYSTEM_CONSTANTS.rotorToSensorRatio,
                    Conversions.mpsToRPS(simState.velocity(), ElevatorConstants.PULLEY_CIRCUMFERENCE.getMeters(), ElevatorConstants.SUBSYSTEM_CONSTANTS.rotorToSensorRatio),
                    simState.supplyVoltage()));
    }
}
