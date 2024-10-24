package frc2025.subsystems.elevator;

import dashboard.Ligament;
import data.Length;
import drivers.TalonFXSubsystem;
import edu.wpi.first.math.geometry.Rotation2d;
import frc2025.RobotContainer;
import frc2025.RobotState;
import frc2025.logging.Logger;
import java.util.function.DoubleSupplier;
import math.Conversions;

public class Elevator extends TalonFXSubsystem {

  private final Ligament elevatorLig =
      new Ligament()
          .withStaticAngle(Rotation2d.fromDegrees(80))
          .withDynamicLength(
              () -> getMeasuredHeight().plus(ElevatorConstants.HOME_HEIGHT_FROM_FLOOR),
              () ->
                  Length.fromRotations(
                          getGoal().target().getAsDouble(), ElevatorConstants.PULLEY_CIRCUMFERENCE)
                      .plus(ElevatorConstants.HOME_HEIGHT_FROM_FLOOR));

  public Elevator(TalonFXSubsystemConfiguration constants) {
    super(constants, ElevatorGoal.TRACKING);

    simFeedforwardSup = () -> 0.3;
    resetPosition(0.0);
  }

  public enum ElevatorGoal implements TalonFXSubsystemGoal {
    HOME_INTAKE(() -> 0.0, ControlType.MOTION_MAGIC_POSITION),
    TRAP_INTAKE(() -> 3.41, ControlType.MOTION_MAGIC_POSITION),
    SUB(() -> 3.13, ControlType.MOTION_MAGIC_POSITION),
    SUB_DEFENDED(() -> ElevatorConstants.MAX_HEIGHT, ControlType.MOTION_MAGIC_POSITION),
    AMP(() -> 19.71, ControlType.MOTION_MAGIC_POSITION),
    TRAP(() -> ElevatorConstants.MAX_HEIGHT, ControlType.MOTION_MAGIC_POSITION),
    EJECT(() -> 5.43, ControlType.MOTION_MAGIC_POSITION),
    TRACKING(
        () ->
            RobotContainer.getRobotState() == null
                ? HOME_INTAKE.target.getAsDouble()
                : RobotState.getActiveShotParameters().shootState().getElevatorHeight(),
        ControlType.MOTION_MAGIC_POSITION);

    public final DoubleSupplier target;

    public final ControlType controlType;

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

  public Ligament getLigament() {
    return elevatorLig;
  }

  public Length getMeasuredHeight() {
    return Length.fromRotations(getPosition(), ElevatorConstants.PULLEY_CIRCUMFERENCE);
  }

  public Length getSetpointHeight() {
    return Length.fromRotations(getSetpoint(), ElevatorConstants.PULLEY_CIRCUMFERENCE);
  }

  @Override
  public synchronized void setSimState(double position, double velocity) {
    super.setSimState(
        Conversions.linearDistanceToRotations(
                Length.fromMeters(position), ElevatorConstants.PULLEY_CIRCUMFERENCE)
            * ElevatorConstants.GEAR_RATIO,
        Conversions.mpsToRPS(
            velocity,
            ElevatorConstants.PULLEY_CIRCUMFERENCE.getMeters(),
            ElevatorConstants.GEAR_RATIO));
  }

  @Override
  public void periodic() {
    super.periodic();
    Logger.log(logPrefix + "Height", getMeasuredHeight().getInches());
  }
}
