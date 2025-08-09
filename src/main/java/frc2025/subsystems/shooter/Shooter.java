package frc2025.subsystems.shooter;

import drivers.TalonFXSubsystem;
import frc2025.Robot;
import frc2025.RobotContainer;
import frc2025.RobotState;
import java.util.function.DoubleSupplier;

public class Shooter extends TalonFXSubsystem {

  public Shooter(TalonFXSubsystemConfiguration constants) {
    super(constants, ShooterGoal.TRACKING);

    if (Robot.isSimulation()) {
      simFeedforwardSup =
          () -> ShooterConstants.SIM_FEEDFORWARD.calculate(getVelocity(), getSetpoint());
    }
  }

  public enum ShooterGoal implements TalonFXSubsystemGoal {
    IDLE(() -> 0.0, ControlType.VOLTAGE),
    SUB(() -> 3000, ControlType.VELOCITY),
    TRACKING(
        () ->
            RobotContainer.getRobotState() == null
                ? 0.0
                : RobotState.getActiveShotParameters().shootState().getVelocityRPM(),
        ControlType.VELOCITY);

    public final DoubleSupplier target;

    public final ControlType controlType;

    private ShooterGoal(DoubleSupplier targetRPM, ControlType controlType) {
      this.target = () -> targetRPM.getAsDouble() / 60.0;
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

    @Override
    public DoubleSupplier feedForward() {
      return () -> 0.0;
    }
  }

  @Override
  public synchronized double getVelocity() {
      return Robot.isSimulation() ? getSetpoint() : super.getVelocity(); 
  }
}
