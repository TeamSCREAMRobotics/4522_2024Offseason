package frc2024.subsystems.elevator;

import com.SCREAMLib.data.Length;
import com.SCREAMLib.drivers.TalonFXSubsystem;
import com.SCREAMLib.math.Conversions;
import frc2024.RobotState;
import java.util.function.DoubleSupplier;
import lombok.Getter;

public class Elevator extends TalonFXSubsystem {

  public Elevator(TalonFXSubsystemConstants constants) {
    super(constants, ElevatorGoal.HOME_INTAKE);

    simFeedforwardSup = () -> 0.3;
  }

  public enum ElevatorGoal implements TalonFXSubsystemGoal {
    HOME_INTAKE(() -> 0.0, ControlType.MOTION_MAGIC_POSITION),
    TRAP_INTAKE(() -> 3.41, ControlType.MOTION_MAGIC_POSITION),
    SUB(() -> 3.13, ControlType.MOTION_MAGIC_POSITION),
    SUB_DEFENDED(() -> 21.75, ControlType.MOTION_MAGIC_POSITION),
    AMP(() -> 19.71, ControlType.MOTION_MAGIC_POSITION),
    TRAP(() -> 21.75, ControlType.MOTION_MAGIC_POSITION),
    EJECT(() -> 5.43, ControlType.MOTION_MAGIC_POSITION),
    TRACKING(
        () -> RobotState.getActiveShotParameters().get().shootState().getElevatorHeight(),
        ControlType.MOTION_MAGIC_POSITION);

    @Getter DoubleSupplier target;

    @Getter ControlType controlType;

    private ElevatorGoal(DoubleSupplier targetHeightInches, ControlType controlType) {
      this.target =
          () ->
              Conversions.linearDistanceToRotations(
                      Length.fromInches(targetHeightInches.getAsDouble()),
                      ElevatorConstants.PULLEY_CIRCUMFERENCE);
      this.controlType = controlType;
    }

    @Override
    public DoubleSupplier target() {
      return target;
    }

    @Override
    public ControlType controlType() {
      return controlType;
    }
  }

  public Length getMeasuredHeight() {
    return Length.fromRotations(getPosition(), ElevatorConstants.PULLEY_CIRCUMFERENCE);
  }

  public Length getSetpointHeight(){
    return Length.fromRotations(getSetpoint(), ElevatorConstants.PULLEY_CIRCUMFERENCE);
  }

  @Override
  public void setSimState(double position, double velocity) {
    super.setSimState(
        Conversions.linearDistanceToRotations(
                Length.fromMeters(position), ElevatorConstants.PULLEY_CIRCUMFERENCE)
            * ElevatorConstants.SUBSYSTEM_CONSTANTS.rotorToSensorRatio,
        Conversions.mpsToRPS(
            velocity,
            ElevatorConstants.PULLEY_CIRCUMFERENCE.getMeters(),
            ElevatorConstants.SUBSYSTEM_CONSTANTS.rotorToSensorRatio));
  }
}
