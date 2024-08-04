package frc2024.subsystems.shooter;

import com.SCREAMLib.drivers.TalonFXSubsystem;
import com.SCREAMLib.drivers.TalonFXSubsystemGoal;
import edu.wpi.first.math.controller.SimpleMotorFeedforward;
import frc2024.Robot;
import frc2024.RobotState;
import java.util.function.DoubleSupplier;
import lombok.Getter;

public class Shooter extends TalonFXSubsystem {

  private SimpleMotorFeedforward simff;

  public Shooter(TalonFXSubsystemConstants constants) {
    super(constants, ShooterGoal.TRACKING);

    if (Robot.isSimulation()) {
      simff = ShooterConstants.SIM_FEEDFORWARD;
      simFeedforward = () -> simff.calculate(getVelocity(), getSetpoint(), constants.simPeriodSec);
    }
  }

  public enum ShooterGoal implements TalonFXSubsystemGoal {
    SUB(() -> 3000, ControlType.VELOCITY),
    TRACKING(
        () -> RobotState.getActiveShotParameters().get().shootState().getVelocityRPM(),
        ControlType.VELOCITY);

    @Getter DoubleSupplier targetRPS;

    @Getter ControlType controlType;

    private ShooterGoal(DoubleSupplier targetRPM, ControlType controlType) {
      targetRPS = () -> targetRPM.getAsDouble() / 60.0;
      this.controlType = controlType;
    }

    @Override
    public DoubleSupplier target() {
      return targetRPS;
    }

    @Override
    public ControlType controlType() {
      return controlType;
    }
  }
}
