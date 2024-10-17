package frc2025.subsystems.drivetrain;

import com.SCREAMLib.pid.ScreamPIDConstants;
import com.SCREAMLib.util.PPUtil;
import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.controllers.PathFollowingController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.util.Units;
import frc2025.subsystems.drivetrain.generated.TunerConstants;

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

  public static final ModuleConfig MODULE_CONFIG =
      new ModuleConfig(
          Units.inchesToMeters(1.97), MAX_SPEED, 1.4, DCMotor.getKrakenX60(1), 80.0, 1);

  public static final RobotConfig ROBOT_CONFIG =
      new RobotConfig(
          Units.lbsToKilograms(150), 6.883, MODULE_CONFIG, TunerConstants.TRACK_WIDTH.getMeters());

  public static final PathFollowingController PATH_FOLLOWING_CONTROLLER =
      new PPHolonomicDriveController(
          PPUtil.screamPIDConstantsToPPConstants(PATH_TRANSLATION_CONSTANTS),
          PPUtil.screamPIDConstantsToPPConstants(PATH_ROTATION_CONSTANTS));
}
