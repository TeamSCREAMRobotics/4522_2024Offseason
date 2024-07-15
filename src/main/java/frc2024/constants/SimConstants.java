package frc2024.constants;

import java.util.ArrayList;
import java.util.List;

import com.team4522.lib.util.AllianceFlipUtil;

import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;

public class SimConstants {

    public static final double EDGE_WIDTH = Units.inchesToMeters(10);
    
    public static final double MECH_WIDTH = Units.inchesToMeters(26 + EDGE_WIDTH);
    public static final double MECH_HEIGHT = Units.inchesToMeters(42 + EDGE_WIDTH);

    public static final double ELEVATOR_X = Units.inchesToMeters(14.329305 + EDGE_WIDTH / 2.0);

    public static final Mechanism2d ACTUAL_MECHANISM = new Mechanism2d(SimConstants.MECH_WIDTH, SimConstants.MECH_HEIGHT);
    public static final Mechanism2d SETPOINT_MECHANISM = new Mechanism2d(SimConstants.MECH_WIDTH, SimConstants.MECH_HEIGHT);

    public static final int MAX_SIM_NOTES = 10;
}
