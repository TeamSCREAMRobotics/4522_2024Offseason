package com.team4522.lib.util;

import java.util.function.DoubleSupplier;
import java.util.function.Supplier;

import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.util.HolonomicPathFollowerConfig;
import com.team4522.lib.pid.ScreamPIDConstants;

import edu.wpi.first.math.MathUtil;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.controller.ProfiledPIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Quaternion;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.geometry.Twist2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.trajectory.TrapezoidProfile.Constraints;

/**
 *  Various utility methods 
 * */
public class ScreamUtil {
    public static final double EPSILON = 1e-3;

	public static Rotation2d boundRotation(Rotation2d rotation){
		return new Rotation2d(MathUtil.angleModulus(rotation.getRadians()));
	}

	public static Rotation2d boundRotation0_360(Rotation2d rotation){
		rotation = boundRotation(rotation);
		if(rotation.getDegrees() < 0)return Rotation2d.fromDegrees(rotation.getDegrees() + 360.0);
		return rotation;
	}

	public static Rotation2d getTangent(Translation2d start, Translation2d end){
		Translation2d dist = end.minus(start);
		return new Rotation2d(dist.getX(), dist.getY());
	}

    public static boolean epsilonEquals(double a, double b, final double epsilon) {
        return (a - epsilon <= b) && (a + epsilon >= b);
    }

    public static boolean epsilonEquals(double a, double b) {
        return epsilonEquals(a, b, EPSILON);
    }

    public static boolean epsilonEquals(int a, int b, int epsilon) {
        return (a - epsilon <= b) && (a + epsilon >= b);
    }

	public double getStallTorque(double stallTorque, double freeSpeed, double speed){
		return -stallTorque/freeSpeed * speed + stallTorque;
	}
	
	public static Twist2d chassisSpeedsToTwist2d(ChassisSpeeds chassisSpeeds){		
        return new Twist2d(chassisSpeeds.vxMetersPerSecond, chassisSpeeds.vyMetersPerSecond, chassisSpeeds.omegaRadiansPerSecond);
	}

	public static boolean epsilonEquals(final Twist2d twist, final Twist2d other, double epsilon) {
        return ScreamUtil.epsilonEquals(twist.dx, other.dx, epsilon) &&
               ScreamUtil.epsilonEquals(twist.dy, other.dy, epsilon) &&
               ScreamUtil.epsilonEquals(twist.dtheta, other.dtheta, epsilon);
    }

	public static boolean epsilonEquals(final Twist2d twist, final Twist2d other) {
        return epsilonEquals(twist, other, EPSILON);
    }

	public static Twist2d getPoseLog(Pose2d transform){
        final double dtheta = transform.getRotation().getRadians();
        final double half_dtheta = 0.5 * dtheta;
        final double cos_minus_one = transform.getRotation().getCos() - 1.0;
        double halftheta_by_tan_of_halfdtheta;
        if (Math.abs(cos_minus_one) < EPSILON) {
            halftheta_by_tan_of_halfdtheta = 1.0 - 1.0 / 12.0 * dtheta * dtheta;
        } else {
            halftheta_by_tan_of_halfdtheta = -(half_dtheta * transform.getRotation().getSin()) / cos_minus_one;
        }
        final Translation2d translation_part = transform.getTranslation()
                .rotateBy(new Rotation2d(halftheta_by_tan_of_halfdtheta, -half_dtheta));
        return new Twist2d(translation_part.getX(), translation_part.getY(), dtheta);
	}

    public static PPHolonomicDriveController createHolonomicDriveController(HolonomicPathFollowerConfig config){
        return new PPHolonomicDriveController(config.translationConstants, config.rotationConstants, config.period, config.maxModuleSpeed, config.driveBaseRadius);
    }
    
    public static Rotation2d calculateAngleToPoint(Translation2d current, Translation2d target){
        double targetX = target.getX() - current.getX();
        double targetY = target.getY() - current.getY();
        return Rotation2d.fromRadians(Math.atan2(targetY, targetX));
      }

    public static double calculateDistanceToTranslation(Translation2d current, Translation2d target){
        return current.getDistance(target);
    }

    public static double average(double... nums){
        if(nums.length == 0) return 0.0;
        
        double sum = 0.0;
        for(double num : nums){
            sum += num;
        }
        return sum / nums.length;
    }

    public static double pythagorean(double a, double b){
        return Math.sqrt(Math.pow(a, 2) + Math.pow(b, 2));
    }

    public static double mapRange(double value, double fromLow, double fromHigh, double toLow, double toHigh) {
        if (fromHigh - fromLow == 0) {
            throw new IllegalArgumentException("Input range has zero width");
        }
    
        return toLow + ((value - fromLow) * (toHigh - toLow)) / (fromHigh - fromLow);
    }

    public static boolean valueBetween(double value, double upper, double lower){
        return value < upper && value > lower;
    }

    public static double[] pose2dToArray(Pose2d pose){
        return new double[]{pose.getX(), pose.getY(), pose.getRotation().getDegrees()};
    }

    public static double[] translation3dToArray(Translation3d translation){
        return new double[]{translation.getX(), translation.getY(), translation.getZ(), 0, 0, 0, 0};
    }

    public static double[] translation3dToArray(Translation3d translation, Rotation3d rot){
        Quaternion quat = rot.getQuaternion();
        return new double[]{translation.getX(), translation.getY(), translation.getZ(), quat.getX(), quat.getY(), quat.getZ(), quat.getW()};
    }

    public static Translation3d pose3dArrayToTranslation3d(double[] array){
        return new Translation3d(array[0], array[1], array[2]);
    }

    public static double[] translation3dArrayToNumArray(Translation3d[] translations){
        double[] combinedArr = new double[translations.length * 7];
        int index = 0;

        for (Translation3d translation : translations) {
            double[] arr = translation3dToArray(translation);

            System.arraycopy(arr, 0, combinedArr, index, arr.length);

            index += arr.length;
        }

        return combinedArr;
    }

    public static Translation3d rotatePoint(Translation3d point, Rotation2d yaw) {
        double cosAngle = yaw.getCos();
        double sinAngle = yaw.getSin();

        double newX = point.getX() * cosAngle - point.getY() * sinAngle;
        double newY = point.getX() * sinAngle + point.getY() * cosAngle;
        double newZ = point.getZ();

        return new Translation3d(newX, newY, newZ);
    }

    /* public static void logBasicMotorOutputs(String path, TalonFX motor){
        Logger.recordOutput(path + "/Motor/StatorCurrent", motor.getStatorCurrent().getValueAsDouble());
        Logger.recordOutput(path + "/Motor/SupplyCurrent", motor.getSupplyCurrent().getValueAsDouble());
        Logger.recordOutput(path + "/Motor/TorqueCurrent", motor.getTorqueCurrent().getValueAsDouble());
        Logger.recordOutput(path + "/Motor/AppliedVoltage", motor.getMotorVoltage().getValueAsDouble());
        Logger.recordOutput(path + "/Motor/TempCelcius", motor.getDeviceTemp().getValueAsDouble());
        Logger.recordOutput(path + "/Motor/ControlMode", motor.getControlMode().getValue().toString());
    }

    public static void logServoMotorOutputs(String path, TalonFX motor){
        Logger.recordOutput(path + "/Measured/Position", motor.getPosition().getValueAsDouble());
        Logger.recordOutput(path + "/Measured/Velocity", motor.getVelocity().getValueAsDouble());
        Logger.recordOutput(path + "/Measured/Acceleration", motor.getAcceleration().getValueAsDouble());
    } */
}
