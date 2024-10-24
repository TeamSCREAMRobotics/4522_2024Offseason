package frc2025.subsystems.drivetrain;

import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.signals.NeutralModeValue;
import com.ctre.phoenix6.swerve.SwerveDrivetrain;
import com.ctre.phoenix6.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.swerve.SwerveModule;
import com.ctre.phoenix6.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.swerve.SwerveRequest;
import com.pathplanner.lib.auto.AutoBuilder;
import com.pathplanner.lib.util.DriveFeedforwards;
import com.pathplanner.lib.util.swerve.SwerveSetpoint;
import com.pathplanner.lib.util.swerve.SwerveSetpointGenerator;
import drivers.PhoenixSwerveHelper;
import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.VecBuilder;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import edu.wpi.first.units.measure.Force;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj2.command.Command;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Subsystem;
import frc2025.constants.Constants;
import frc2025.constants.FieldConstants;
import frc2025.logging.Logger;
import frc2025.subsystems.vision.Vision;
import java.util.Optional;
import java.util.function.DoubleSupplier;
import java.util.function.Supplier;
import lombok.Getter;
import math.ScreamMath;
import util.AllianceFlipUtil;
import util.RunnableUtil.RunOnce;
import util.ScreamUtil;
import vision.LimelightHelpers.PoseEstimate;
import vision.LimelightVision;

/**
 * Class that extends the Phoenix SwerveDrivetrain class and implements subsystem so it can be used
 * in command-based projects easily.
 */
public class Drivetrain extends SwerveDrivetrain implements Subsystem {
  private double lastSimTime;

  private RunOnce operatorPerspectiveApplier = new RunOnce();

  @Getter private final PhoenixSwerveHelper helper;

  private final SwerveSetpointGenerator setpointGenerator;
  private SwerveSetpoint previousSetpoint;

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

    AutoBuilder.configure(
        this::getPose,
        this::seedFieldRelative,
        this::getRobotRelativeSpeeds,
        this::setChassisSpeeds,
        DrivetrainConstants.PATH_FOLLOWING_CONTROLLER,
        DrivetrainConstants.ROBOT_CONFIG,
        AllianceFlipUtil.shouldFlip(),
        this);

    setpointGenerator =
        new SwerveSetpointGenerator(
            DrivetrainConstants.ROBOT_CONFIG, DrivetrainConstants.MAX_AZIMUTH_VEL_RADS);
    previousSetpoint =
        new SwerveSetpoint(
            getRobotRelativeSpeeds(), getModuleStates(), DriveFeedforwards.zeros(m_modules.length));

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
    previousSetpoint =
        setpointGenerator.generateSetpoint(previousSetpoint, speeds, Constants.PERIOD_SEC);
    Force[] xFeedforwards = previousSetpoint.feedforwards().robotRelativeForcesX();
    Force[] yFeedforwards = previousSetpoint.feedforwards().robotRelativeForcesY();
    setControl(
        helper
            .getApplyChassisSpeeds(speeds)
            .withWheelForceFeedforwardsX(xFeedforwards)
            .withWheelForceFeedforwardsY(yFeedforwards));
  }

  public void setNeutralModes(NeutralModeValue driveMode, NeutralModeValue steerMode) {
    for (SwerveModule mod : getModules()) {
      mod.getDriveMotor().setNeutralMode(driveMode);
      mod.getSteerMotor().setNeutralMode(steerMode);
    }
  }

  public void resetHeading() {
    seedFieldRelative(new Pose2d(getPose().getTranslation(), AllianceFlipUtil.getFwdHeading()));
  }

  public SwerveRequest getNoteAssistRequest(
      Translation2d translation, double angularVelocity, Optional<Translation2d> closestNote) {

    if (closestNote.isPresent()) {
      DrivetrainConstants.NOTE_ASSIST_CONTROLLER.reset();
      Translation2d robotPosition = getPose().getTranslation();
      Translation2d robotToNote = closestNote.get().minus(robotPosition);
      double noteDistance = robotToNote.getNorm();

      if (noteDistance < 4.5) {
        robotToNote = robotToNote.rotateBy(getHeading().unaryMinus());
        translation =
            translation
                .rotateBy(getHeading().unaryMinus())
                .minus(
                    new Translation2d(
                        0,
                        DrivetrainConstants.NOTE_ASSIST_CONTROLLER.calculate(
                            robotToNote.getY(), 0)));
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
        .beforeStarting(() -> DrivetrainConstants.NOTE_ASSIST_CONTROLLER.reset());
  }

  public boolean getWithinAngleThreshold(Rotation2d targetAngle, Rotation2d threshold) {
    return ScreamUtil.withinAngleThreshold(targetAngle, getHeading(), threshold);
  }

  public SwerveModule[] getModules() {
    return m_modules;
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
    return getState().Speeds;
  }

  public double getLinearVelocity() {
    return ScreamMath.getLinearVelocity(getFieldRelativeSpeeds());
  }

  public SwerveModuleState[] getModuleStates() {
    SwerveModuleState[] states = new SwerveModuleState[m_modules.length];
    for (int i = 0; i < m_modules.length; i++) {
      states[i] = m_modules[i].getCurrentState();
    }
    return states;
  }

  public DoubleSupplier calculatePathRotationFeedback(Rotation2d targetHeading) {
    return () ->
        DrivetrainConstants.PATH_OVERRIDE_CONTROLLER.calculate(
            getHeading().getRadians(), targetHeading.getRadians());
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
        || Math.abs(getPigeon2().getAngularVelocityZWorld().getValueAsDouble()) > 540
        || getLinearVelocity() > 3.0) {
      return;
    }
    addVisionMeasurement(
        poseEstimate.pose,
        poseEstimate.timestampSeconds,
        VecBuilder.fill(
            Math.pow(0.5, poseEstimate.tagCount) * poseEstimate.avgTagDist * 2,
            Math.pow(0.5, poseEstimate.tagCount) * poseEstimate.avgTagDist * 2,
            9999999));
  }

  @Override
  public void periodic() {
    attemptToSetPerspective();
    updateFromVision(
        LimelightVision.getPoseEstimate_MT2(
            Vision.SHOOTER_LIMELIGHT,
            getHeading().getDegrees(),
            getPigeon2().getAngularVelocityZWorld().getValueAsDouble()));
    if (getCurrentCommand() != null) {
      Logger.log("RobotState/Subsystems/Drivetrain/ActiveCommand", getCurrentCommand().getName());
    }
  }

  public void attemptToSetPerspective() {
    operatorPerspectiveApplier.runOnceWhenTrueThenWhenChanged(
        () -> setOperatorPerspectiveForward(AllianceFlipUtil.getFwdHeading()),
        DriverStation.getAlliance().isPresent(),
        DriverStation.getAlliance().orElse(null));
  }
}
