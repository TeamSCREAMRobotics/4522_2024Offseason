package frc2024.dashboard;

import java.util.function.Supplier;

import org.littletonrobotics.junction.Logger;

import com.team4522.lib.util.Length;

import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.smartdashboard.Mechanism2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismLigament2d;
import edu.wpi.first.wpilibj.smartdashboard.MechanismRoot2d;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj.util.Color;
import edu.wpi.first.wpilibj.util.Color8Bit;
import edu.wpi.first.wpilibj2.command.SubsystemBase;
import frc2024.constants.SimConstants;

public class MechanismVisualizer extends SubsystemBase{
    
    public static Mechanism2d MEASURED_MECHANISM;
    public static Mechanism2d SETPOINT_MECHANISM;

    public static void setDimensions(double width, double height){
        MEASURED_MECHANISM = new Mechanism2d(width, height);
        SETPOINT_MECHANISM = new Mechanism2d(width, height);
    }

    private final MechanismRoot2d measuredRoot;
    private final MechanismRoot2d setpointRoot;

    private final MechanismLigament2d measuredLig;
    private final MechanismLigament2d setpointLig;

    private Supplier<Length> measuredLength = () -> new Length();
    private Supplier<Rotation2d> measuredAngle = () -> new Rotation2d();
    private Supplier<Length> setpointLength = () -> new Length();
    private Supplier<Rotation2d> setpointAngle = () -> new Rotation2d();

    private Supplier<Translation2d> position = () -> new Translation2d();

    public MechanismVisualizer(String key){
        if(MEASURED_MECHANISM == null || SETPOINT_MECHANISM == null){
            MEASURED_MECHANISM = new Mechanism2d(0, 0);
            SETPOINT_MECHANISM = new Mechanism2d(0, 0);
            DriverStation.reportError("Set mechanism dimensions before creating MechanismVisualizer!", true);
        }

        measuredRoot = MechanismVisualizer.MEASURED_MECHANISM.getRoot(key + " Actual", 0, 0);
        setpointRoot = MechanismVisualizer.SETPOINT_MECHANISM.getRoot(key + " Setpoint", 0, 0);

        measuredLig = measuredRoot.append(new MechanismLigament2d("Actual", 0.0, 0.0, 6, new Color8Bit(Color.kRed)));
        setpointLig = setpointRoot.append(new MechanismLigament2d("Setpoint", 0.0, 0.0, 6, new Color8Bit(Color.kGreen)));
    }

    public MechanismVisualizer withStaticAngle(Rotation2d angle){
        this.measuredAngle = () -> angle;
        this.setpointAngle = () -> angle;
        return this;
    }

    public MechanismVisualizer withDynamicAngle(Supplier<Rotation2d> measured, Supplier<Rotation2d> setpoint){
        this.measuredAngle = measured;
        this.setpointAngle = setpoint;
        return this;
    }

    public MechanismVisualizer withStaticLength(Length length){
        this.measuredLength = () -> length;
        this.setpointLength = () -> length;
        return this;
    }

    public MechanismVisualizer withDynamicLength(Supplier<Length> measured, Supplier<Length> setpoint){
        this.measuredLength = measured;
        this.setpointLength = setpoint;
        return this;
    }

    public MechanismVisualizer withStaticPosition(Translation2d position){
        this.position = () -> position;
        return this;
    }

    public MechanismVisualizer withDynamicPosition(Supplier<Translation2d> position){
        this.position = position;
        return this;
    }

    public void setAngle(Rotation2d angle){
        this.measuredAngle = () -> angle;
        this.setpointAngle = () -> angle;
    }

    public void setAngle(Rotation2d measured, Rotation2d setpoint){
        this.measuredAngle = () -> measured;
        this.setpointAngle = () -> setpoint;
    }

    public void setLength(Length length){
        this.measuredLength = () -> length;
        this.setpointLength = () -> length;
    }

    public void setLength(Length measured, Length setpoint){
        this.measuredLength = () -> measured;
        this.setpointLength = () -> setpoint;
    }

    public void setPosition(Translation2d position){
        this.position = () -> position;
    }

    private static void outputToDashboard(){
        Logger.recordOutput("RobotState/Mechanisms/Measured", MechanismVisualizer.MEASURED_MECHANISM);
        Logger.recordOutput("RobotState/Mechanisms/Setpoint", MechanismVisualizer.SETPOINT_MECHANISM);
    }

    @Override
    public void periodic(){
        this.setpointLig.setLength(setpointLength.get().getMeters());
        this.setpointLig.setAngle(setpointAngle.get().getDegrees());
        this.measuredLig.setLength(measuredLength.get().getMeters());
        this.measuredLig.setAngle(measuredAngle.get().getDegrees());
        this.setpointRoot.setPosition(position.get().getX(), position.get().getY());
        this.measuredRoot.setPosition(position.get().getX(), position.get().getY());
        MechanismVisualizer.outputToDashboard();
    }
}
