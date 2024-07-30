package frc2024.controlboard;

import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import com.team4522.lib.util.AllianceFlipUtil;
import com.team4522.lib.util.ScreamUtil;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.filter.SlewRateLimiter;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc2024.RobotContainer;

public class Controlboard {
    public static final CommandXboxController driveController = new CommandXboxController(0);
    public static final CommandXboxController operatorController = new CommandXboxController(1);
    public static final Buttonboard buttonboard = new Buttonboard(2, 3);

    public static final double STICK_DEADBAND = 0.1;
    public static final double TRIGGER_DEADBAND = 0.1;

    public static boolean fieldCentric = true;
    static{
        driveController.start().onTrue(Commands.runOnce(() -> fieldCentric = !fieldCentric));
    }

    public static Supplier<Translation2d> getRawTranslation(){
        return () -> 
            new Translation2d(
                driveController.getLeftY(),
                driveController.getLeftX());
    }

    public static Supplier<Translation2d> getTranslation(){
        return () -> 
            new Translation2d(
                -MathUtil.applyDeadband(AllianceFlipUtil.Number(driveController.getLeftY(), -driveController.getLeftY()), STICK_DEADBAND) * (getSlowMode().getAsBoolean() ? 0.5 : 1),
                -MathUtil.applyDeadband(AllianceFlipUtil.Number(driveController.getLeftX(), -driveController.getLeftX()), STICK_DEADBAND) * (getSlowMode().getAsBoolean() ? 0.5 : 1));
    }

    public static DoubleSupplier getRotation(){
        return () -> Math.pow(MathUtil.applyDeadband(driveController.getRightX(), STICK_DEADBAND), 2) * -Math.signum(driveController.getRightX());
    }

    public static BooleanSupplier getFieldCentric(){
        return () -> fieldCentric;
    }

    public static BooleanSupplier getSlowMode(){
        return driveController.leftTrigger(TRIGGER_DEADBAND).or(driveController.leftBumper());
    }
}
