package frc2024.subsystems.shooter;

import com.SCREAMLib.drivers.TalonFXSubsystem;
import frc2024.Robot;
import frc2024.RobotContainer;
import frc2024.RobotState;
import java.util.function.DoubleSupplier;
import lombok.Getter;

public class Shooter extends TalonFXSubsystem {

  public Shooter(TalonFXSubsystemConstants constants) {
    super(constants, ShooterGoal.TRACKING);

    if (Robot.isSimulation()) {
      simFeedforwardSup =
          () ->
              ShooterConstants.SIM_FEEDFORWARD.calculate(
                  getVelocity(), getSetpoint(), constants.simPeriodSec);
    }
  }

  public enum ShooterGoal implements TalonFXSubsystemGoal {
    IDLE(() -> 0.0, ControlType.VOLTAGE),
    SUB(() -> 3000, ControlType.VELOCITY),
    TRACKING(
        () ->
            RobotContainer.getRobotState() == null
                ? IDLE.getTarget().getAsDouble()
                : RobotState.getActiveShotParameters().get().shootState().getVelocityRPM(),
        ControlType.VELOCITY);

    @Getter DoubleSupplier target;

    @Getter ControlType controlType;

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
  }
}
