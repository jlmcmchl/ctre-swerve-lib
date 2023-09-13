/*
 * CONFIDENTIAL - DO NOT SHARE
 *
 * Copyright (C) Cross The Road Electronics.  All rights reserved.
 * License information can be found in CTRE_LICENSE.txt
 * For support and suggestions contact support@ctr-electronics.com or file
 * an issue tracker at https://github.com/CrossTheRoadElec/Phoenix-Releases
 */
package frc.robot.ctre;

import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.mechanisms.swerve.utility.PhoenixPIDController;

import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModuleState;

/**
 * Container for all the Swerve Requests. Use this to find all applicable swerve
 * drive requests.
 * <p>
 * This is also an interface common to all swerve drive control requests that
 * allow the
 * request to calculate the state to apply to the modules.
 */
public interface SwerveRequest {

    /*
     * Contains everything the control requests need to calculate the module state
     */
    public class SwerveControlRequestParameters {
        public SwerveDriveKinematics kinematics;
        public Pose2d currentPose;
        public double timestamp;
        public Translation2d[] swervePositions;
    }

    public StatusCode apply(SwerveControlRequestParameters parameters, SwerveModule... modulesToApply);

    /**
     * Sets the swerve drive module states to point inward on the
     * robot in an "X" fashion, creating a natural brake which will
     * oppose any motion.
     */
    public class SwerveDriveBrake implements SwerveRequest {

        /**
         * True to use open-loop control while stopped.
         */
        public boolean IsOpenLoop = true;

        public StatusCode apply(SwerveControlRequestParameters parameters, SwerveModule... modulesToApply) {

            for (int i = 0; i < modulesToApply.length; ++i) {
                SwerveModuleState state = new SwerveModuleState(0, parameters.swervePositions[i].getAngle());
                modulesToApply[i].apply(state, IsOpenLoop);
            }

            return StatusCode.OK;
        }

        public SwerveDriveBrake withIsOpenLoop(boolean isOpenLoop) {
            this.IsOpenLoop = isOpenLoop;
            return this;
        }
    }

    /**
     * Drives the swerve drivetrain in a field-centric manner.
     * <p>
     * When users use this request, they specify the direction the robot should
     * travel
     * oriented against the field, and the rate at which their robot should rotate
     * about the center of the robot.
     * <p>
     * An example scenario is that the robot is oriented to the east,
     * the VelocityX is +5 m/s, VelocityY is 0 m/s, and RotationRate is 0.5 rad/s.
     * In this scenario, the robot would drive northward at 5 m/s and
     * turn counterclockwise at 0.5 rad/s.
     */
    public class FieldCentric implements SwerveRequest {
        /**
         * The velocity in the X direction.
         * X is defined as forward according to WPILib convention,
         * so this determines how fast to travel forward.
         */
        public double VelocityX = 0;
        /**
         * The velocity in the Y direction.
         * Y is defined as to the left according to WPILib convention,
         * so this determines how fast to travel to the left.
         */
        public double VelocityY = 0;
        /**
         * The angular rate to rotate at.
         * Angular rate is defined as counterclockwise positive,
         * so this determines how fast to turn counterclockwise.
         * <p>
         * This is in radians per second
         */
        public double RotationalRate = 0;

        /**
         * True to use open-loop control when driving.
         */
        public boolean IsOpenLoop = true;

        public StatusCode apply(SwerveControlRequestParameters parameters, SwerveModule... modulesToApply) {
            ChassisSpeeds speeds = ChassisSpeeds.fromFieldRelativeSpeeds(VelocityX, VelocityY, RotationalRate,
                    parameters.currentPose.getRotation());

            var states = parameters.kinematics.toSwerveModuleStates(speeds, new Translation2d());

            for (int i = 0; i < modulesToApply.length; ++i) {
                modulesToApply[i].apply(states[i], IsOpenLoop);
            }

            return StatusCode.OK;
        }

        public FieldCentric withVelocityX(double velocityX) {
            this.VelocityX = velocityX;
            return this;
        }

        public FieldCentric withVelocityY(double velocityY) {
            this.VelocityY = velocityY;
            return this;
        }

        public FieldCentric withRotationalRate(double rotationalRate) {
            this.RotationalRate = rotationalRate;
            return this;
        }

        public FieldCentric withIsOpenLoop(boolean isOpenLoop) {
            this.IsOpenLoop = isOpenLoop;
            return this;
        }
    }

    /**
     * Drives the swerve drivetrain in a field-centric manner, maintaining a
     * specified heading
     * angle to ensure the robot is facing the desired direction
     * <p>
     * When users use this request, they specify the direction the robot should
     * travel
     * oriented against the field, and the direction the robot should be facing.
     * <p>
     * An example scenario is that the robot is oriented to the east,
     * the VelocityX is +5 m/s, VelocityY is 0 m/s, and TargetDirection is 180
     * degrees.
     * In this scenario, the robot would drive northward at 5 m/s and
     * turn clockwise to a target of 180 degrees.
     * <p>
     * This control request is especially useful for autonomous control, where the
     * robot should be facing a changing direction throughout the motion.
     */
    public class FieldCentricFacingAngle implements SwerveRequest {
        /**
         * The velocity in the X direction.
         * X is defined as forward according to WPILib convention,
         * so this determines how fast to travel forward.
         */
        public double VelocityX = 0;
        /**
         * The velocity in the Y direction.
         * Y is defined as to the left according to WPILib convention,
         * so this determines how fast to travel to the left.
         */
        public double VelocityY = 0;
        /**
         * The desired direction to face.
         * 0 Degrees is defined as in the direction of the X axis.
         * So a TargetDirection of 90 degrees will point along the Y axis, or to the
         * left.
         */
        public Rotation2d TargetDirection = new Rotation2d();

        /**
         * True to use open-loop control when driving.
         */
        public boolean IsOpenLoop = true;

        /**
         * The PID controller used to maintain the desired heading.
         * Users can specify the PID gains to change how aggressively to maintain
         * heading.
         * <p>
         * This PID controller operates on heading radians and outputs a target
         * rotational rate in radians per second.
         */
        public PhoenixPIDController HeadingController = new PhoenixPIDController(0, 0, 0);

        public StatusCode apply(SwerveControlRequestParameters parameters, SwerveModule... modulesToApply) {
            double rotationRate = HeadingController.calculate(parameters.currentPose.getRotation().getRadians(),
                    TargetDirection.getRadians(), parameters.timestamp);

            ChassisSpeeds speeds = ChassisSpeeds.fromFieldRelativeSpeeds(VelocityX, VelocityY, rotationRate,
                    parameters.currentPose.getRotation());

            var states = parameters.kinematics.toSwerveModuleStates(speeds, new Translation2d());

            for (int i = 0; i < modulesToApply.length; ++i) {
                modulesToApply[i].apply(states[i], IsOpenLoop);
            }

            return StatusCode.OK;
        }

        public FieldCentricFacingAngle withVelocityX(double velocityX) {
            this.VelocityX = velocityX;
            return this;
        }

        public FieldCentricFacingAngle withVelocityY(double velocityY) {
            this.VelocityY = velocityY;
            return this;
        }

        public FieldCentricFacingAngle withTargetDirection(Rotation2d targetDirection) {
            this.TargetDirection = targetDirection;
            return this;
        }

        public FieldCentricFacingAngle withIsOpenLoop(boolean isOpenLoop) {
            this.IsOpenLoop = isOpenLoop;
            return this;
        }
    }

    /**
     * Does nothing to the swerve module state. This is the default state of a newly
     * created swerve drive mechanism.
     */
    public class Idle implements SwerveRequest {

        /**
         * True to use open-loop control while stopped.
         */
        public boolean IsOpenLoop = true;

        public StatusCode apply(SwerveControlRequestParameters parameters, SwerveModule... modulesToApply) {

            return StatusCode.OK;
        }

        public Idle withIsOpenLoop(boolean isOpenLoop) {
            this.IsOpenLoop = isOpenLoop;
            return this;
        }
    }

    /**
     * Sets the swerve drive modules to point to a specified direction.
     */
    public class PointWheelsAt implements SwerveRequest {

        /**
         * The direction to point the modules toward.
         * This direction is still optimized to what the module was previously at.
         */
        public Rotation2d ModuleDirection = new Rotation2d();
        /**
         * True to use open-loop control while stopped.
         */
        public boolean IsOpenLoop = true;

        public StatusCode apply(SwerveControlRequestParameters parameters, SwerveModule... modulesToApply) {

            for (int i = 0; i < modulesToApply.length; ++i) {
                SwerveModuleState state = new SwerveModuleState(0, ModuleDirection);
                modulesToApply[i].apply(state, IsOpenLoop);
            }

            return StatusCode.OK;
        }

        public PointWheelsAt withModuleDirection(Rotation2d moduleDirection) {
            this.ModuleDirection = moduleDirection;
            return this;
        }

        public PointWheelsAt withIsOpenLoop(boolean isOpenLoop) {
            this.IsOpenLoop = isOpenLoop;
            return this;
        }
    }

    /**
     * Drives the swerve drivetrain in a robot-centric manner.
     * <p>
     * When users use this request, they specify the direction the robot should
     * travel
     * oriented against the robot itself, and the rate at which their
     * robot should rotate about the center of the robot.
     * <p>
     * An example scenario is that the robot is oriented to the east,
     * the VelocityX is +5 m/s, VelocityY is 0 m/s, and RotationRate is 0.5 rad/s.
     * In this scenario, the robot would drive eastward at 5 m/s and
     * turn counterclockwise at 0.5 rad/s.
     */
    public class RobotCentric implements SwerveRequest {
        /**
         * The velocity in the X direction.
         * X is defined as forward according to WPILib convention,
         * so this determines how fast to travel forward.
         */
        public double VelocityX = 0;
        /**
         * The velocity in the Y direction.
         * Y is defined as to the left according to WPILib convention,
         * so this determines how fast to travel to the left.
         */
        public double VelocityY = 0;
        /**
         * The angular rate to rotate at.
         * Angular rate is defined as counterclockwise positive,
         * so this determines how fast to turn counterclockwise.
         * <p>
         * This is in radians per second
         */
        public double RotationalRate = 0;

        /**
         * True to use open-loop control when driving.
         */
        public boolean IsOpenLoop = true;

        public StatusCode apply(SwerveControlRequestParameters parameters, SwerveModule... modulesToApply) {
            ChassisSpeeds speeds = new ChassisSpeeds(VelocityX, VelocityY, RotationalRate);

            var states = parameters.kinematics.toSwerveModuleStates(speeds, new Translation2d());

            for (int i = 0; i < modulesToApply.length; ++i) {
                modulesToApply[i].apply(states[i], IsOpenLoop);
            }

            return StatusCode.OK;
        }

        public RobotCentric withVelocityX(double velocityX) {
            this.VelocityX = velocityX;
            return this;
        }

        public RobotCentric withVelocityY(double velocityY) {
            this.VelocityY = velocityY;
            return this;
        }

        public RobotCentric withRotationalRate(double rotationalRate) {
            this.RotationalRate = rotationalRate;
            return this;
        }

        public RobotCentric withIsOpenLoop(boolean isOpenLoop) {
            this.IsOpenLoop = isOpenLoop;
            return this;
        }
    }
}
