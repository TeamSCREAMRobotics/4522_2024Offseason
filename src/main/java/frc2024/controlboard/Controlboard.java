package frc2024.controlboard;

import com.SCREAMLib.util.AllianceFlipUtil;
import com.SCREAMLib.util.ScreamUtil;
import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandXboxController;
import frc2024.subsystems.swerve.SwerveConstants;
import java.util.function.BooleanSupplier;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

public class Controlboard {
  public static final CommandXboxController driveController = new CommandXboxController(0);
  public static final CommandXboxController operatorController = new CommandXboxController(1);
  public static final Buttonboard buttonboard = new Buttonboard(2, 3);

  public static final double STICK_DEADBAND = 0.05;
  public static final double TRIGGER_DEADBAND = 0.1;
  public static final Rotation2d SNAP_TO_POLE_THRESHOLD = Rotation2d.fromDegrees(4.0);

  public static boolean fieldCentric = true;

  static {
    driveController.start().onTrue(Commands.runOnce(() -> fieldCentric = !fieldCentric));
  }

  public static double applyPower(double value, int power) {
    return Math.pow(value, power) * (power % 2 == 0 ? Math.signum(value) : 1);
  }

  public static Supplier<Translation2d> getRawTranslation() {
    return () -> new Translation2d(driveController.getLeftY(), driveController.getLeftX());
  }

  public static Supplier<Translation2d> getTranslation() {
    return () ->
        snapTranslationToPole(
            new Translation2d(
                    applyPower(
                        -MathUtil.applyDeadband(
                            AllianceFlipUtil.get(
                                driveController.getLeftY(), -driveController.getLeftY()),
                            STICK_DEADBAND),
                        2),
                    applyPower(
                        -MathUtil.applyDeadband(
                            AllianceFlipUtil.get(
                                driveController.getLeftX(), -driveController.getLeftX()),
                            STICK_DEADBAND),
                        2))
                .times(SwerveConstants.MAX_SPEED));
  }

  public static Translation2d snapTranslationToPole(Translation2d translation) {
    for (int i = 0; i < 4; i++) {
      if (ScreamUtil.withinAngleThreshold(
          Rotation2d.fromDegrees(i * 90.0), translation.getAngle(), SNAP_TO_POLE_THRESHOLD)) {
        return new Translation2d(translation.getNorm(), Rotation2d.fromDegrees(i * 90.0));
      }
    }
    return translation;
  }

  public static DoubleSupplier getRotation() {
    return () ->
        applyPower(-MathUtil.applyDeadband(driveController.getRightX(), STICK_DEADBAND), 3)
            * SwerveConstants.MAX_ANGULAR_SPEED;
  }

  public static BooleanSupplier getFieldCentric() {
    return () -> fieldCentric;
  }

  public static BooleanSupplier getSlowMode() {
    return driveController.leftTrigger(TRIGGER_DEADBAND).or(driveController.leftBumper());
  }
}
