package frc2024.subsystems;

import java.util.ArrayList;
import java.util.List;
import java.util.function.DoubleSupplier;

import org.opencv.core.Point;

import com.ctre.phoenix6.Utils;
import com.team4522.lib.drivers.TalonFXSubsystem;
import com.team4522.lib.math.Conversions;
import com.team4522.lib.util.Length;
import com.team4522.lib.util.ScreamUtil;

import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.system.plant.DCMotor;
import edu.wpi.first.math.util.Units;
import edu.wpi.first.wpilibj.RobotController;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.simulation.DCMotorSim;
import edu.wpi.first.wpilibj.simulation.FlywheelSim;
import edu.wpi.first.wpilibj.smartdashboard.SmartDashboard;
import edu.wpi.first.wpilibj2.command.Command;
import frc2024.RobotContainer;
import frc2024.RobotState;
import frc2024.constants.Constants;
import frc2024.constants.ElevatorConstants;
import frc2024.constants.PivotConstants;
import frc2024.constants.ShooterConstants;
import lombok.Getter;
import lombok.Setter;

public class Shooter extends TalonFXSubsystem{

    public Shooter(TalonFXSubystemConstants constants) {
        super(constants);
    }

    public final FlywheelSim sim = new FlywheelSim(DCMotor.getFalcon500(1), 1.0, 0.00599676919909 + 0.0001);
    public final PIDController simController = new PIDController(10.0, 0.0, 0.0);

    public enum Goal{
        SUB(() -> 3000),
        SUB_DEFENDED(() -> 3000),
        AUTO(() -> RobotContainer.getRobotState().getShotParameters().shootState().velocityRPM());

        @Getter
        DoubleSupplier targetRPS;
        
        private Goal(DoubleSupplier targetRPM){
            targetRPS = () -> targetRPM.getAsDouble() / 60.0;
        }
    }

    @Getter @Setter
    private Goal goal = Goal.AUTO;

    public Command setGoalCommand(Goal goal){
        return run(() -> setGoal(goal));
    }

    Timer timer = new Timer();
    int test = 0;
    boolean test2 = true;
    @Override
    public void periodic() {
        super.periodic();
        if(!ShooterConstants.updateFromTuner && !Utils.isSimulation()){
            setSetpointVelocity(getGoal().getTargetRPS().getAsDouble());
        } else {
            double inputVoltage = simController.calculate(getVelocity(), getGoal().getTargetRPS().getAsDouble());
            sim.update(Constants.SIM_PERIOD_SEC);
            sim.setInputVoltage(inputVoltage);
            setSimState(new SimState(0.0, sim.getAngularVelocityRPM() / 60.0, inputVoltage), getGoal().getTargetRPS().getAsDouble(), true);
        }
        SmartDashboard.putNumberArray("Note Trajectory", calculateTrajectory(Rotation2d.fromRotations(RobotContainer.getSubsystems().pivot().getPosition()).minus(PivotConstants.RELATIVE_ENCODER_TO_HORIZONTAL).getDegrees(), Conversions.falconRPSToMechanismMPS(RobotContainer.getSubsystems().shooter().getVelocity(), ShooterConstants.WHEEL_CIRCUMFERENCE.getMeters(), 1.0) * 0.8, RobotContainer.getSubsystems().drivetrain().getPose()));
    }

    public static double[] calculateTrajectory(double launchAngle, double initialVelocity, Pose2d pose) {
        int numSteps = 15; // Number of steps to divide the trajectory into
        double[] trajectoryPoints = new double[(numSteps + 1) * 7];
        double launchAngleRad = Math.toRadians(launchAngle); // Convert angle to radians
        double totalTime = (2 * initialVelocity * Math.sin(launchAngleRad)) / Constants.GRAVITY; // Total time of flight

        double timeInterval = totalTime / numSteps; // Time interval for each step

        int index = 0; // Index to fill the trajectoryPoints array

        for (int i = 0; i <= numSteps; i++) {
            double time = i * timeInterval;
            double x = initialVelocity * Math.cos(launchAngleRad) * time;
            double y = (initialVelocity * Math.sin(launchAngleRad) * time) - (0.5 * Constants.GRAVITY * time * time);
            double z = 0; // Assuming the trajectory is in the x-y plane
            if (y < 0) {
                y = 0; // Ensure the projectile doesn't go below ground level
            }

            Length absElevHeight = RobotContainer.getSubsystems().elevator().getHeight().plus(ElevatorConstants.HOME_HEIGHT_FROM_FLOOR);
            Translation2d pivotRootPos = new Translation2d(absElevHeight.minus(PivotConstants.AXLE_DISTANCE_FROM_ELEVATOR_TOP).getMeters(), Rotation2d.fromDegrees(80));

            // Rotate the point relative to the pose
            Translation3d rotatedPoint = rotatePoint(new Translation3d(x, z, y).plus(new Translation3d(pivotRootPos.getX(), 0.0, pivotRootPos.getY())), pose.getRotation().getDegrees());

            // Translate the point relative to the pose
            double relX = rotatedPoint.getX() + pose.getX();
            double relY = rotatedPoint.getY() + pose.getY();
            double relZ = rotatedPoint.getZ();

            trajectoryPoints[index++] = relX;
            trajectoryPoints[index++] = relY;
            trajectoryPoints[index++] = relZ;
            trajectoryPoints[index++] = 0.0;
            trajectoryPoints[index++] = 0.0;
            trajectoryPoints[index++] = 0.0;
            trajectoryPoints[index++] = 0.0;
        }

        return trajectoryPoints;
    }

    // Rotate a point around the z-axis (yaw)
    private static Translation3d rotatePoint(Translation3d point, double yaw) {
        double angleRad = Math.toRadians(yaw);
        double cosAngle = Math.cos(angleRad);
        double sinAngle = Math.sin(angleRad);

        double newX = point.getX() * cosAngle - point.getY() * sinAngle;
        double newY = point.getX() * sinAngle + point.getY() * cosAngle;
        double newZ = point.getZ();

        return new Translation3d(newX, newY, newZ);
    }
}
