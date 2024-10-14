package frc2024.subsystems.drivetrain;

import com.SCREAMLib.pid.ScreamPIDConstants;
import com.SCREAMLib.util.PPUtil;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.ReplanningConfig;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import frc2024.constants.Constants;
import frc2024.subsystems.drivetrain.generated.TunerConstants;

public final class DrivetrainConstants {
  public static final double MAX_SPEED = TunerConstants.SPEED_12V_MPS;
  public static final double MAX_ANGULAR_SPEED = 8.0;

  public static final double NOTE_ASSIST_KP = 2.0;

  public static final ScreamPIDConstants PATH_TRANSLATION_CONSTANTS =
      new ScreamPIDConstants(10.0, 0.0, 0.0);
  public static final ScreamPIDConstants PATH_ROTATION_CONSTANTS =
      new ScreamPIDConstants(3.0, 0.0, 0.0);
  public static final ProfiledPIDController HEADING_CONTROLLER =
      new ProfiledPIDController(8.0, 0, 0, new Constraints(30, 15));

  static {
    HEADING_CONTROLLER.enableContinuousInput(-Math.PI, Math.PI);
  }

  public static final ScreamPIDConstants HEADING_CORRECTION_CONSTANTS =
      new ScreamPIDConstants(8.0, 0.0, 0.0);

  public static final HolonomicPathFollowerConfig PATH_FOLLOWER_CONFIG =
      new HolonomicPathFollowerConfig(
          PPUtil.screamPIDConstantsToPPConstants(PATH_TRANSLATION_CONSTANTS),
          PPUtil.screamPIDConstantsToPPConstants(PATH_ROTATION_CONSTANTS),
          MAX_SPEED,
          TunerConstants.DRIVE_BASE_RADIUS,
          new ReplanningConfig(),
          Constants.PERIOD_SEC);
}
