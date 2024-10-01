package frc2024.dashboard;

import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Dashboard {

  public static class DashboardValue {
    private String key;
    private boolean defaultValue;
    private boolean value;

    public DashboardValue(String key, boolean defaultValue) {
      this.key = key;
      this.defaultValue = defaultValue;
      this.value = defaultValue;
      SmartDashboard.putBoolean(key, SmartDashboard.getBoolean(key, defaultValue));
    }

    public void setDefault(boolean defaultValue) {
      this.defaultValue = defaultValue;
    }

    public void set(boolean value) {
      SmartDashboard.putBoolean(key, value);
    }

    public boolean get() {
      return SmartDashboard.getBoolean(key, defaultValue);
    }
  }
}
