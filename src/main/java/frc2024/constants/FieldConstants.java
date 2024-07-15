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

  public static final class StagingLocations {
    public static final double centerlineX = FIELD_DIMENSIONS.getX() / 2.0;

    public static final double centerlineFirstY = Units.inchesToMeters(29.638);
    public static final double centerlineSeparationY = Units.inchesToMeters(66);
    public static final double spikeX = Units.inchesToMeters(114);
    public static final double spikeFirstY = Units.inchesToMeters(161.638);
    public static final double spikeSeparationY = Units.inchesToMeters(57);

    public static final Translation2d[] centerlineTranslations = new Translation2d[5];
    public static final Translation2d[] spikeTranslations = new Translation2d[3];

    static {
      for (int i = 0; i < centerlineTranslations.length; i++) {
        centerlineTranslations[i] =
            new Translation2d(centerlineX, centerlineFirstY + (i * centerlineSeparationY));
      }
    }
    static {
      for (int i = 0; i < spikeTranslations.length; i++) {
        spikeTranslations[i] = new Translation2d(spikeX, spikeFirstY + (i * spikeSeparationY));
      }
    }
  }
}
