package frc2025.subsystems.drivetrain;

import com.pathplanner.lib.config.ModuleConfig;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.controllers.PathFollowingController;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;
import edu.wpi.first.math.util.Units;
import frc2025.subsystems.drivetrain.generated.TunerConstants;
import pid.ScreamPIDConstants;
import util.PPUtil;

public final class DrivetrainConstants {
  public static final double MAX_SPEED = TunerConstants.SPEED_12V_MPS;
  public static final double MAX_ANGULAR_SPEED_RADS = 8.0;

  public static final double MAX_AZIMUTH_VEL_RADS = Units.rotationsToRadians(10);

  public static final int NUM_MODULES = 4;

  public static final ScreamPIDConstants PATH_TRANSLATION_CONSTANTS =
      new ScreamPIDConstants(10.0, 0.0, 0.0);
  public static final ScreamPIDConstants PATH_ROTATION_CONSTANTS =
      new ScreamPIDConstants(3.0, 0.0, 0.0);
  public static final ProfiledPIDController HEADING_CONTROLLER =
      new ProfiledPIDController(8.0, 0, 0, new Constraints(30, 15));

  public static final PIDController PATH_OVERRIDE_CONTROLLER =
      DrivetrainConstants.PATH_ROTATION_CONSTANTS.getPIDController(-Math.PI, Math.PI);

  public static final PIDController NOTE_ASSIST_CONTROLLER = new PIDController(4.0, 0, 0.01);

  static {
    HEADING_CONTROLLER.enableContinuousInput(-Math.PI, Math.PI);
  }

  public static final ScreamPIDConstants HEADING_CORRECTION_CONSTANTS =
      new ScreamPIDConstants(8.0, 0.0, 0.0);

  public static final ModuleConfig MODULE_CONFIG =
      new ModuleConfig(Units.inchesToMeters(1.97), MAX_SPEED, 1.4, DCMotor.getKrakenX60(1), 85, 1);

  public static final RobotConfig ROBOT_CONFIG =
      new RobotConfig(
          Units.lbsToKilograms(150), 6.883, MODULE_CONFIG, TunerConstants.TRACK_WIDTH.getMeters());

  public static final PathFollowingController PATH_FOLLOWING_CONTROLLER =
      new PPHolonomicDriveController(
          PPUtil.screamPIDConstantsToPPConstants(PATH_TRANSLATION_CONSTANTS),
          PPUtil.screamPIDConstantsToPPConstants(PATH_ROTATION_CONSTANTS));
}
