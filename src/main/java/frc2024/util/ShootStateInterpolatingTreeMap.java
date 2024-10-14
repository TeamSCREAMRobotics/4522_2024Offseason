package frc2024.util;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.interpolation.InterpolatingDoubleTreeMap;
import lombok.Getter;
import lombok.Setter;

public class ShootStateInterpolatingTreeMap {
  private InterpolatingDoubleTreeMap angleInterpolator = new InterpolatingDoubleTreeMap();
  private InterpolatingDoubleTreeMap heightInterpolator = new InterpolatingDoubleTreeMap();
  private InterpolatingDoubleTreeMap velocityInterpolator = new InterpolatingDoubleTreeMap();

  public void put(double distance, ShootState shootState) {
    angleInterpolator.put(distance, shootState.getPivotAngle().getDegrees());
    heightInterpolator.put(distance, shootState.getElevatorHeight());
    velocityInterpolator.put(distance, shootState.getVelocityRPM());
  }

  public ShootState get(double distance) {
    return new ShootState(
        Rotation2d.fromDegrees(angleInterpolator.get(distance)),
        heightInterpolator.get(distance),
        velocityInterpolator.get(distance));
  }

  public static class ShootState {

    @Getter @Setter Rotation2d pivotAngle;

    @Getter @Setter double elevatorHeight, velocityRPM;

    public ShootState(Rotation2d pivotAngle, double elevatorHeight, double velocityRPM) {
      this.pivotAngle = pivotAngle;
      this.elevatorHeight = elevatorHeight;
      this.velocityRPM = velocityRPM;
    }

    public ShootState() {
      this.pivotAngle = new Rotation2d();
      this.elevatorHeight = 0;
      this.velocityRPM = 0;
    }
  }
}
