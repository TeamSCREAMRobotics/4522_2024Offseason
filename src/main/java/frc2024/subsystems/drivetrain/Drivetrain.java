package frc2024.subsystems.drivetrain;

import com.SCREAMLib.drivers.PhoenixSwerveHelper;
import com.SCREAMLib.math.ScreamMath;
import com.SCREAMLib.util.AllianceFlipUtil;
import com.SCREAMLib.util.ScreamUtil;
import com.SCREAMLib.vision.LimelightHelpers.PoseEstimate;
import com.SCREAMLib.vision.LimelightVision;
import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrain;
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModule;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveRequest;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.pathplanner.lib.auto.AutoBuilder;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc2024.constants.FieldConstants;
import frc2024.logging.Logger;
import frc2024.subsystems.vision.Vision;
import java.util.Optional;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
import lombok.Getter;

/**
 * Class that extends the Phoenix SwerveDrivetrain class and implements subsystem so it can be used
 * in command-based projects easily.
 */
public class Drivetrain extends SwerveDrivetrain implements Subsystem {
  private double lastSimTime;

  @Getter private final PhoenixSwerveHelper helper;

  public Drivetrain(
      SwerveDrivetrainConstants driveTrainConstants,
      double OdometryUpdateFrequency,
      SwerveModuleConstants... modules) {
    super(driveTrainConstants, OdometryUpdateFrequency, modules);

    CommandScheduler.getInstance().registerSubsystem(this);

    helper =
        new PhoenixSwerveHelper(
            this::getPose,
            DrivetrainConstants.MAX_SPEED,
            DrivetrainConstants.HEADING_CORRECTION_CONSTANTS,
            DrivetrainConstants.HEADING_CORRECTION_CONSTANTS);

    AutoBuilder.configureHolonomic(
        this::getPose,
        this::seedFieldRelative,
        this::getRobotRelativeSpeeds,
        this::setChassisSpeeds,
        DrivetrainConstants.PATH_FOLLOWER_CONFIG,
        AllianceFlipUtil.shouldFlip(),
        this);

    System.out.println("[Init] Drivetrain initialization complete!");
  }

  public Command applyRequest(Supplier<SwerveRequest> requestSupplier) {
    return run(() -> this.setControl(requestSupplier.get()))
        .withName("Drivetrain: applyRequest(" + requestSupplier.get().toString() + ")");
  }

  public void updateSimState() {
    final double currentTime = Utils.getCurrentTimeSeconds();
    double deltaTime = currentTime - lastSimTime;
    lastSimTime = currentTime;

    /* use the measured time delta, get battery voltage from WPILib */
    updateSimState(deltaTime, RobotController.getBatteryVoltage());
  }

  private void setChassisSpeeds(ChassisSpeeds speeds) {
    setControl(helper.getApplyChassisSpeeds(speeds));
  }

  public void setNeutralModes(NeutralModeValue driveMode, NeutralModeValue steerMode) {
    for (SwerveModule mod : getModules()) {
      mod.getDriveMotor().setNeutralMode(driveMode);
      mod.getSteerMotor().setNeutralMode(steerMode);
    }
  }

  public void resetHeading() {
    seedFieldRelative(
        new Pose2d(getPose().getTranslation(), AllianceFlipUtil.getForwardRotation()));
  }

  PIDController test = new PIDController(4.0, 0, 0.01);

  public SwerveRequest getNoteAssistRequest(
      Translation2d translation, double angularVelocity, Optional<Translation2d> closestNote) {

    if (closestNote.isPresent()) {
      test.reset();
      Translation2d robotPosition = getPose().getTranslation();
      Translation2d robotToNote = closestNote.get().minus(robotPosition);
      double noteDistance = robotToNote.getNorm();

      if (noteDistance < 4.5) {
        robotToNote = robotToNote.rotateBy(getHeading().unaryMinus());
        translation =
            translation
                .rotateBy(getHeading().unaryMinus())
                .minus(new Translation2d(0, test.calculate(robotToNote.getY(), 0)));
        return helper.getHeadingCorrectedFieldCentric(
            translation.rotateBy(getHeading()), angularVelocity);
      }
    }

    return helper.getHeadingCorrectedFieldCentric(translation, angularVelocity);
  }

  public Command getNoteAssistCommand(
      Supplier<Translation2d> translation,
      DoubleSupplier angularVelocity,
      Supplier<Optional<Translation2d>> closestNote) {
    return this.applyRequest(
            () ->
                getNoteAssistRequest(
                    translation.get(), angularVelocity.getAsDouble(), closestNote.get()))
        .beforeStarting(() -> test.reset());
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

  public double getLinearVelocity() {
    return ScreamMath.getLinearVelocity(getFieldRelativeSpeeds());
  }

  public void stop() {
    setControl(new SwerveRequest.Idle());
  }

  public void optimizeModuleUtilization(SwerveModule module) {
    module.getDriveMotor().optimizeBusUtilization();
    module.getSteerMotor().optimizeBusUtilization();
    module.getCANcoder().optimizeBusUtilization();
  }

  @Override
  public void addVisionMeasurement(
      Pose2d visionRobotPoseMeters,
      double timestampSeconds,
      Matrix<N3, N1> visionMeasurementStdDevs) {
    super.addVisionMeasurement(visionRobotPoseMeters, timestampSeconds, visionMeasurementStdDevs);
    Logger.log("RobotState/Subsystems/Drivetrain/ActivePoseEstimate", visionRobotPoseMeters);
  }

  public void updateFromVision(PoseEstimate poseEstimate) {
    if (poseEstimate == null
        || poseEstimate.tagCount == 0
        || !FieldConstants.Zones.FIELD_AREA.contains(poseEstimate.pose)
        || Math.abs(getPigeon2().getRate()) > 540
        || getLinearVelocity() > 3.0) {
      return;
    }
    addVisionMeasurement(
        poseEstimate.pose,
        poseEstimate.timestampSeconds,
        VecBuilder.fill(
            Math.pow(0.8, poseEstimate.tagCount) * poseEstimate.avgTagDist * 2,
            Math.pow(0.8, poseEstimate.tagCount) * poseEstimate.avgTagDist * 2,
            9999999));
  }

  @Override
  public void periodic() {
    updateFromVision(
        LimelightVision.getPoseEstimate_MT2(
            Vision.SHOOTER_LIMELIGHT, getHeading().getDegrees(), getPigeon2().getRate()));
    if (getCurrentCommand() != null) {
      Logger.log("RobotState/Subsystems/Drivetrain/ActiveCommand", getCurrentCommand().getName());
    }
  }
}
