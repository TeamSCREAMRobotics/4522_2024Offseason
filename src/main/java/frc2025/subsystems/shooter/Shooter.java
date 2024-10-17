package frc2025.subsystems.shooter;

import com.SCREAMLib.drivers.TalonFXSubsystem;
import edu.wpi.first.units.Units;
import frc2025.Robot;
import frc2025.RobotContainer;
import frc2025.RobotState;
import java.util.function.DoubleSupplier;
import lombok.Getter;

public class Shooter extends TalonFXSubsystem {

  public Shooter(TalonFXSubsystemConstants constants) {
    super(constants, ShooterGoal.TRACKING);

    if (Robot.isSimulation()) {
      simFeedforwardSup =
          () ->
              ShooterConstants.SIM_FEEDFORWARD
                  .calculate(
                      Units.RotationsPerSecond.of(getVelocity()),
                      Units.RotationsPerSecond.of(getVelocity()))
                  .baseUnitMagnitude();
    }
  }

  public enum ShooterGoal implements TalonFXSubsystemGoal {
    IDLE(() -> 0.0, ControlType.VOLTAGE),
    SUB(() -> 3000, ControlType.VELOCITY),
    TRACKING(
        () ->
            RobotContainer.getRobotState() == null
                ? IDLE.getTarget().getAsDouble()
                : RobotState.getActiveShotParameters().shootState().getVelocityRPM(),
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
