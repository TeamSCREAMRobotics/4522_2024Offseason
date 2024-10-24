package frc2025.constants;

import data.Length;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import zones.RectangularPoseArea;

public final class FieldConstants {

  public static final Translation2d FIELD_DIMENSIONS = new Translation2d(16.541, 8.211);

  public static final Length NOTE_DIAMETER = Length.fromInches(14.0);

  public static final Length NOTE_HEIGHT = Length.fromInches(1.0);

  public static final class Zones {
    public static final RectangularPoseArea FIELD_AREA =
        new RectangularPoseArea(new Translation2d(0, 0), FIELD_DIMENSIONS);
    public static final RectangularPoseArea CENTER_AREA =
        new RectangularPoseArea(
            new Translation2d(5.85, 0), new Translation2d(10.7, FIELD_DIMENSIONS.getY()));
    public static final RectangularPoseArea BLUE_WING_AREA =
        new RectangularPoseArea(
            new Translation2d(0.0, 0.0), new Translation2d(5.87, FIELD_DIMENSIONS.getY()));
    public static final RectangularPoseArea RED_WING_AREA =
        new RectangularPoseArea(new Translation2d(10.7, 0.0), FIELD_DIMENSIONS);
  }

  public static final class PointsOfInterest {
    public static final Translation2d BLUE_WING_FEED_TRANSLATION = new Translation2d(2.0, 6.6);
    public static final Translation2d BLUE_CENTER_FEED_TRANSLATION = new Translation2d(6.85, 6.65);
  }

  public static final class ScoringLocations {
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
        new Translation3d(0.0, Units.inchesToMeters(238.815), Units.inchesToMeters(78.324));
    public static final Translation3d SPEAKER_BOTTOM_LEFT =
        new Translation3d(0.0, Units.inchesToMeters(197.765), Units.inchesToMeters(78.324));

    public static final double SPEAKER_HEIGHT_OFFSET = -Units.inchesToMeters(0.0);
    public static final Translation3d SPEAKER_OPENING =
        SPEAKER_BOTTOM_LEFT
            .interpolate(SPEAKER_TOP_RIGHT, 0.5)
            .plus(new Translation3d(0, 0, SPEAKER_HEIGHT_OFFSET));
  }

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
