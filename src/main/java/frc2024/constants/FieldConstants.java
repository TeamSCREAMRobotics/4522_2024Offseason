package frc2024.constants;

import com.team4522.lib.util.RectanglePoseArea;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;

public final class FieldConstants{

  public static final Translation2d FIELD_DIMENSIONS = new Translation2d(16.541, 8.211);
  
  public static final Translation3d SPEAKER_TOP_RIGHT = 
    new Translation3d(
      Units.inchesToMeters(18.055), 
      Units.inchesToMeters(238.815),
      Units.inchesToMeters(83.091));
  public static final Translation3d SPEAKER_TOP_LEFT =
    new Translation3d(
      Units.inchesToMeters(18.055),
      Units.inchesToMeters(197.765),
      Units.inchesToMeters(83.091));
  public static final Translation3d SPEAKER_BOTTOM_RIGHT =
    new Translation3d(
      0.0, 
      Units.inchesToMeters(238.815), 
      Units.inchesToMeters(78.324));
  public static final Translation3d SPEAKER_BOTTOM_LEFT =
    new Translation3d(
      0.0, 
      Units.inchesToMeters(197.765), 
      Units.inchesToMeters(78.324));
  
  public static final Translation3d SPEAKER_OPENING = SPEAKER_BOTTOM_LEFT.interpolate(SPEAKER_TOP_RIGHT, 0.5);
  public static final Translation2d SPEAKER_GOAL_OFFSET_RIGHT = new Translation2d(Units.inchesToMeters(12.5), Units.inchesToMeters(9.0));
  public static final Translation2d SPEAKER_GOAL_OFFSET_LEFT = new Translation2d(Units.inchesToMeters(5.0), Units.inchesToMeters(9.0));
  public static final Translation2d SPEAKER_GOAL_OFFSET_CENTER = new Translation2d(Units.inchesToMeters(4.0), 0);
  
  public static final RectanglePoseArea FIELD_AREA = new RectanglePoseArea(new Translation2d(0, 0), FIELD_DIMENSIONS); //Blue origin
  public static final RectanglePoseArea WING_POSE_AREA = new RectanglePoseArea(new Translation2d(10.4, 0.0), FIELD_DIMENSIONS); //Blue origin
}
