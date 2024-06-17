package frc2024.constants;

import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.pathplanner.lib.util.ReplanningConfig;
import com.team4522.lib.pid.ScreamPIDConstants;

import frc2024.subsystems.swerve.generated.TunerConstants;

public final class SwerveConstants{
    public static final double MAX_SPEED = TunerConstants.SPEED_12V_MPS;
    public static final double MAX_ANGULAR_SPEED = 8.0;

    public static final ScreamPIDConstants PATH_TRANSLATION_CONSTANTS = new ScreamPIDConstants(10.0, 0.0, 0.0);
    public static final ScreamPIDConstants PATH_ROTATION_CONSTANTS = new ScreamPIDConstants(10.0, 0.0, 0.0);

    public static final ScreamPIDConstants HEADING_CONSTANTS = new ScreamPIDConstants(0.1, 0.0, 0.001);

    public static final ScreamPIDConstants SNAP_CONSTANTS = new ScreamPIDConstants(10, 0, 0.8);

    public static final HolonomicPathFollowerConfig PATH_FOLLOWER_CONFIG = new HolonomicPathFollowerConfig(
      PATH_TRANSLATION_CONSTANTS.getPathPlannerPIDConstants(), 
      PATH_ROTATION_CONSTANTS.getPathPlannerPIDConstants(), 
      MAX_SPEED, 
      TunerConstants.DRIVE_BASE_RADIUS, 
      new ReplanningConfig(), 
      Constants.PERIOD_SEC);
  }