package frc2024.subsystems.swerve;

import com.SCREAMLib.util.AllianceFlipUtil;
import com.SCREAMLib.util.PhoenixSwerveUtil;
import com.SCREAMLib.util.ScreamUtil;
import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrain;
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Notifier;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc2024.Robot;
import frc2024.constants.Constants;
import java.util.function.Supplier;
import lombok.Getter;

/**
 * Class that extends the Phoenix SwerveDrivetrain class and implements subsystem so it can be used
 * in command-based projects easily.
 */
public class Drivetrain extends SwerveDrivetrain implements Subsystem {
  private Notifier simNotifier = null;
  private double lastSimTime;

  @Getter private final PhoenixSwerveUtil util;

  public Drivetrain(
      SwerveDrivetrainConstants driveTrainConstants,
      double OdometryUpdateFrequency,
      SwerveModuleConstants... modules) {
    super(driveTrainConstants, OdometryUpdateFrequency, modules);

    CommandScheduler.getInstance().registerSubsystem(this);

    if (Robot.isSimulation()) {
      startSimThread();
    }

    /*
     * for(SwerveModule mod : getModules()){ new RunnableUtil.RunUntil().tryUntil(()
     * -> optimizeModuleUtilization(mod)); }
     */

    util =
        new PhoenixSwerveUtil(
            this::getHeading, SwerveConstants.MAX_ANGULAR_SPEED, SwerveConstants.SNAP_CONSTANTS);

    AutoBuilder.configureHolonomic(
        this::getPose,
        this::seedFieldRelative,
        this::getRobotRelativeSpeeds,
        this::setChassisSpeeds,
        SwerveConstants.PATH_FOLLOWER_CONFIG,
        AllianceFlipUtil.shouldFlip(),
        this);

    System.out.println("[Init] Drivetrain Initialization Complete!");
  }

  public Command applyRequest(Supplier<SwerveRequest> requestSupplier) {
    return run(() -> this.setControl(requestSupplier.get()));
  }

  private void startSimThread() {
    lastSimTime = Utils.getCurrentTimeSeconds();

    /* Run simulation at a faster rate so PID gains behave more reasonably */
    simNotifier =
        new Notifier(
            () -> {
              final double currentTime = Utils.getCurrentTimeSeconds();
              double deltaTime = currentTime - lastSimTime;
              lastSimTime = currentTime;

              /* use the measured time delta, get battery voltage from WPILib */
              updateSimState(deltaTime, RobotController.getBatteryVoltage());
            });
    simNotifier.startPeriodic(Constants.SIM_PERIOD_SEC);
  }

  private void setChassisSpeeds(ChassisSpeeds speeds) {
    setControl(util.getApplyChassisSpeeds(speeds));
  }

  public void setNeutralModes(NeutralModeValue driveMode, NeutralModeValue steerMode) {
    for (SwerveModule mod : getModules()) {
      mod.getDriveMotor().setNeutralMode(driveMode);
      mod.getSteerMotor().setNeutralMode(steerMode);
    }
  }

  public boolean getWithinAngleThreshold(Rotation2d targetAngle, Rotation2d threshold) {
    return ScreamUtil.withinAngleThreshold(targetAngle, getHeading(), threshold);
  }

  public SwerveModule[] getModules() {
    return Modules;
  }

  public Pose2d getPose() {
    return getState().Pose;
  }

  public Rotation2d getHeading() {
    return getPose().getRotation();
  }

  public ChassisSpeeds getRobotRelativeSpeeds() {
    return m_kinematics.toChassisSpeeds(getState().ModuleStates);
  }

  public ChassisSpeeds getFieldRelativeSpeeds() {
    return getState().speeds;
  }

  public void optimizeModuleUtilization(SwerveModule module) {
    module.getDriveMotor().optimizeBusUtilization();
    module.getSteerMotor().optimizeBusUtilization();
    module.getCANcoder().optimizeBusUtilization();
  }
}
