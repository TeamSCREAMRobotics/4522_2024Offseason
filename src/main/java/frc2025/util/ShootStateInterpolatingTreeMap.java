package frc2025.util;

import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import lombok.Getter;
import lombok.Setter;

public class ShootStateInterpolatingTreeMap {
  private InterpolatingDoubleTreeMap angleInterpolator = new InterpolatingDoubleTreeMap();
  private InterpolatingDoubleTreeMap heightInterpolator = new InterpolatingDoubleTreeMap();
  private InterpolatingDoubleTreeMap velocityInterpolator = new InterpolatingDoubleTreeMap();

  public void put(double distance, ShootState shootState) {
    angleInterpolator.put(distance, shootState.getPivotAngleDeg());
    heightInterpolator.put(distance, shootState.getElevatorHeight());
    velocityInterpolator.put(distance, shootState.getVelocityRPM());
  }

  public ShootState get(double distance) {
    return new ShootState(
        angleInterpolator.get(distance),
        heightInterpolator.get(distance),
        velocityInterpolator.get(distance));
  }

  public static class ShootState {

    @Getter @Setter double pivotAngleDeg;

    @Getter @Setter double elevatorHeight, velocityRPM;

    public ShootState(double pivotAngleDeg, double elevatorHeight, double velocityRPM) {
      this.pivotAngleDeg = pivotAngleDeg;
      this.elevatorHeight = elevatorHeight;
      this.velocityRPM = velocityRPM;
    }

    public ShootState() {
      this.pivotAngleDeg = 0.0;
      this.elevatorHeight = 0;
      this.velocityRPM = 0;
    }
  }
}
