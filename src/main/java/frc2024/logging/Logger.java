package frc2024.logging;

import dev.doglog.DogLog;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;

public class Logger extends DogLog {
  public static void log(String key, Mechanism2d value) {
    SmartDashboard.putData(key, value);
  }
}
