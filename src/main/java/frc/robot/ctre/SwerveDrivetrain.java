/*
 * CONFIDENTIAL - DO NOT SHARE
 *
 * Copyright (C) Cross The Road Electronics.  All rights reserved.
 * License information can be found in CTRE_LICENSE.txt
 * For support and suggestions contact support@ctr-electronics.com or file
 * an issue tracker at https://github.com/CrossTheRoadElec/Phoenix-Releases
 */
package frc.robot.ctre;

import java.util.concurrent.locks.ReadWriteLock;
import java.util.concurrent.locks.ReentrantReadWriteLock;
import java.util.function.Consumer;

import com.ctre.phoenix6.BaseStatusSignal;
import com.ctre.phoenix6.StatusCode;
import com.ctre.phoenix6.Utils;
import com.ctre.phoenix6.hardware.Pigeon2;
import com.ctre.phoenix6.mechanisms.swerve.SwerveDrivetrainConstants;
import com.ctre.phoenix6.mechanisms.swerve.SwerveModuleConstants;
import com.ctre.phoenix6.unmanaged.Unmanaged;

import edu.wpi.first.math.Matrix;
import edu.wpi.first.math.estimator.SwerveDrivePoseEstimator;
import edu.wpi.first.math.filter.LinearFilter;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.math.kinematics.SwerveModulePosition;
import edu.wpi.first.math.kinematics.SwerveModuleState;
import edu.wpi.first.math.numbers.N1;
import edu.wpi.first.math.numbers.N3;
import frc.robot.ctre.SwerveRequest.SwerveControlRequestParameters;

/**
 * Swerve Drive class utilizing CTR Electronics' Phoenix 6 API.
 * <p>
 * This class handles the kinematics, configuration, and odometry of a
 * swerve drive utilizing CTR Electronics devices. We recommend
 * that users use the Swerve Mechanism Generator in Tuner X to create
 * a template project that demonstrates how to use this class.
 * <p>
 * This class will construct the hardware devices internally, so the user
 * only specifies the constants (IDs, PID gains, gear ratios, etc).
 * Getters for these hardware devices are available.
 * <p>
 * If using the generator, the order in which modules are constructed is
 * Front Left, Front Right, Back Left, Back Right. This means if you need
 * the Back Left module, call {@code getModule(2);} to get the 3rd index
 * (0-indexed) module, corresponding to the Back Left module.
 */
public class SwerveDrivetrain {
    private final int ModuleCount;
    private final double UpdateFrequency;
    private final SwerveModule[] Modules;

    private Pigeon2 m_pigeon2;
    private SwerveDriveKinematics m_kinematics;
    private SwerveDrivePoseEstimator m_odometry;
    private SwerveModulePosition[] m_modulePositions;
    private Translation2d[] m_moduleLocations;
    private OdometryThread m_odometryThread;
    private Rotation2d m_fieldRelativeOffset;

    private SwerveRequest m_requestToApply = new SwerveRequest.Idle();
    private SwerveControlRequestParameters m_requestParameters = new SwerveControlRequestParameters();

    private ReadWriteLock m_stateLock = new ReentrantReadWriteLock();

    private final SimSwerveDrivetrain m_simDrive;
    private final boolean IsOnCANFD;

    /**
     * Plain-Old-Data class holding the state of the swerve drivetrain.
     * This encapsulates most data that is relevant for telemetry or
     * decision-making from the Swerve Drive.
     */
    public class SwerveDriveState {
        public int SuccessfulDaqs;
        public int FailedDaqs;
        public Pose2d Pose;
        public ChassisSpeeds ChassisSpeeds;
        public SwerveModuleState[] ModuleStates;
        public double OdometryPeriod;
    }

    private Consumer<SwerveDriveState> m_telemetryFunction = null;
    private SwerveDriveState m_cachedState = new SwerveDriveState();

    /* Perform swerve module updates in a separate thread to minimize latency */
    private class OdometryThread extends Thread {
        private BaseStatusSignal[] m_allSignals;
        public int SuccessfulDaqs = 0;
        public int FailedDaqs = 0;
        LinearFilter lowpass = LinearFilter.movingAverage(50);
        double lastTime = 0;
        double currentTime = 0;
        double averageLoopTime = 0;

        public OdometryThread() {
            super();
            // 4 signals for each module + 2 for Pigeon2
            m_allSignals = new BaseStatusSignal[(ModuleCount * 4) + 2];
            for (int i = 0; i < ModuleCount; ++i) {
                var signals = Modules[i].getSignals();
                m_allSignals[(i * 4) + 0] = signals[0];
                m_allSignals[(i * 4) + 1] = signals[1];
                m_allSignals[(i * 4) + 2] = signals[2];
                m_allSignals[(i * 4) + 3] = signals[3];
            }
            m_allSignals[m_allSignals.length - 2] = m_pigeon2.getYaw().clone();
            m_allSignals[m_allSignals.length - 1] = m_pigeon2.getAngularVelocityZ().clone();
        }

        @Override
        public void run() {
            /* Make sure all signals update at around 250hz */
            for (var sig : m_allSignals) {
                sig.setUpdateFrequency(UpdateFrequency);
            }
            /* Run as fast as possible, our signals will control the timing */
            while (true) {
                /* Synchronously wait for all signals in drivetrain */
                /* Wait up to twice the period of the update frequency */
                StatusCode status;
                if(IsOnCANFD) {
                    status = BaseStatusSignal.waitForAll(2.0 / UpdateFrequency, m_allSignals);
                } else {
                    try {
                        /* Wait for the signals to update */
                        Thread.sleep((long)((1.0 / UpdateFrequency) * 1000.0));
                    } catch(InterruptedException ex) {}
                    status = BaseStatusSignal.refreshAll(m_allSignals);
                }
                m_stateLock.writeLock().lock();

                lastTime = currentTime;
                currentTime = Utils.getCurrentTimeSeconds();
                averageLoopTime = lowpass.calculate(currentTime - lastTime);

                /* Get status of first element */
                if (status.isOK()) {
                    SuccessfulDaqs++;
                } else {
                    FailedDaqs++;
                }

                /* Now update odometry */
                /* Keep track of the change in azimuth rotations */
                for (int i = 0; i < ModuleCount; ++i) {
                    m_modulePositions[i] = Modules[i].getPosition(false);
                }
                // Assume Pigeon2 is flat-and-level so latency compensation can be performed
                double yawDegrees = BaseStatusSignal.getLatencyCompensatedValue(
                        m_pigeon2.getYaw(), m_pigeon2.getAngularVelocityZ());

                /* Keep track of previous and current pose to account for the carpet vector */
                m_odometry.update(Rotation2d.fromDegrees(yawDegrees), m_modulePositions);

                /* And now that we've got the new odometry, update the controls */
                m_requestParameters.currentPose = m_odometry.getEstimatedPosition()
                        .relativeTo(new Pose2d(0, 0, m_fieldRelativeOffset));
                m_requestParameters.kinematics = m_kinematics;
                m_requestParameters.swervePositions = m_moduleLocations;
                m_requestParameters.timestamp = currentTime;

                m_requestToApply.apply(m_requestParameters, Modules);

                /* Update our cached state with the newly updated data */
                m_cachedState.FailedDaqs = FailedDaqs;
                m_cachedState.SuccessfulDaqs = SuccessfulDaqs;
                m_cachedState.ModuleStates = new SwerveModuleState[Modules.length];
                for (int i = 0; i < Modules.length; ++i) {
                    m_cachedState.ModuleStates[i] = Modules[i].getCurrentState();
                }
                m_cachedState.Pose = m_odometry.getEstimatedPosition();
                m_cachedState.ChassisSpeeds = m_kinematics.toChassisSpeeds(m_cachedState.ModuleStates);
                m_cachedState.OdometryPeriod = averageLoopTime;

                if(m_telemetryFunction != null) {
                    /* Log our state */
                    m_telemetryFunction.accept(m_cachedState);
                }

                m_stateLock.writeLock().unlock();
            }
        }

        public boolean odometryIsValid() {
            return SuccessfulDaqs > 2; // Wait at least 3 daqs before saying the odometry is valid
        }
    }

    /**
     * Constructs a CTRSwerveDrivetrain using the specified constants.
     * <p>
     * This constructs the underlying hardware devices, so user should not construct
     * the devices themselves. If they need the devices, they can access them
     * through
     * getters in the classes.
     *
     * @param driveTrainConstants Drivetrain-wide constants for the swerve drive
     * @param modules             Constants for each specific module
     */
    public SwerveDrivetrain(SwerveDrivetrainConstants driveTrainConstants, SwerveModuleConstants... modules) {
        this(driveTrainConstants, 250, modules);
    }

    /**
     * Constructs a CTRSwerveDrivetrain using the specified constants.
     * <p>
     * This constructs the underlying hardware devices, so user should not construct
     * the devices themselves. If they need the devices, they can access them
     * through
     * getters in the classes.
     *
     * @param driveTrainConstants     Drivetrain-wide constants for the swerve drive
     * @param OdometryUpdateFrequency The frequency to run the odometry loop. If
     *                                unspecified, this is 250 hz.
     * @param modules                 Constants for each specific module
     */
    public SwerveDrivetrain(
            SwerveDrivetrainConstants driveTrainConstants, double OdometryUpdateFrequency,
            SwerveModuleConstants... modules) {
        UpdateFrequency = OdometryUpdateFrequency;
        ModuleCount = modules.length;

        IsOnCANFD = Unmanaged.isNetworkFD(driveTrainConstants.CANbusName);

        m_pigeon2 = new Pigeon2(driveTrainConstants.Pigeon2Id, driveTrainConstants.CANbusName);

        Modules = new SwerveModule[ModuleCount];
        m_modulePositions = new SwerveModulePosition[ModuleCount];
        m_moduleLocations = new Translation2d[ModuleCount];

        int iteration = 0;
        for (SwerveModuleConstants module : modules) {
            Modules[iteration] = new SwerveModule(module, driveTrainConstants.CANbusName,
                    driveTrainConstants.SupportsPro);
            m_moduleLocations[iteration] = new Translation2d(module.LocationX, module.LocationY);
            m_modulePositions[iteration] = Modules[iteration].getPosition(true);

            iteration++;
        }
        m_kinematics = new SwerveDriveKinematics(m_moduleLocations);
        m_odometry = new SwerveDrivePoseEstimator(m_kinematics, new Rotation2d(), m_modulePositions, new Pose2d());

        m_fieldRelativeOffset = new Rotation2d();

        m_simDrive = new SimSwerveDrivetrain(m_moduleLocations, m_pigeon2, driveTrainConstants, modules);

        m_odometryThread = new OdometryThread();
        m_odometryThread.start();
    }

    /**
     * Applies the specified control request to this swerve drivetrain.
     *
     * @param request Request to apply
     */
    public void setControl(SwerveRequest request) {
        try {
            m_stateLock.writeLock().lock();

            m_requestToApply = request;
        } finally {
            m_stateLock.writeLock().unlock();
        }
    }

    /**
     * Zero's this swerve drive's odometry entirely.
     * <p>
     * This will zero the entire odometry, and place the robot at 0,0
     */
    public void tareEverything() {
        try {
            m_stateLock.writeLock().lock();

            for (int i = 0; i < ModuleCount; ++i) {
                Modules[i].resetPosition();
                m_modulePositions[i] = Modules[i].getPosition(true);
            }
            m_odometry.resetPosition(m_pigeon2.getRotation2d(), m_modulePositions, new Pose2d());
        } finally {
            m_stateLock.writeLock().unlock();
        }
    }

    /**
     * Takes the current orientation of the robot and makes it X forward for
     * field-relative
     * maneuvers.
     */
    public void seedFieldRelative() {
        try {
            m_stateLock.writeLock().lock();

            m_fieldRelativeOffset = getState().Pose.getRotation();
        } finally {
            m_stateLock.writeLock().unlock();
        }
    }

    /**
     * Takes the specified location and makes it the current pose for
     * field-relative maneuvers
     *
     * @param location Pose to make the current pose at.
     */
    public void seedFieldRelative(Pose2d location) {
        try {
            m_stateLock.writeLock().lock();

            m_fieldRelativeOffset = location.getRotation();
            m_odometry.resetPosition(location.getRotation(), m_modulePositions, location);
        } finally {
            m_stateLock.writeLock().unlock();
        }
    }

    /**
     * Check if the odometry is currently valid
     *
     * @return True if odometry is valid
     */
    public boolean odometryIsValid() {
        try {
            m_stateLock.readLock().lock();

            return m_odometryThread.odometryIsValid();
        } finally {
            m_stateLock.readLock().unlock();
        }
    }

    /**
     * Get a reference to the module at the specified index.
     * The index corresponds to the module described in the constructor
     *
     * @param index Which module to get
     * @return Reference to SwerveModule
     */
    public SwerveModule getModule(int index) {
        if (index >= Modules.length)
            return null;
        return Modules[index];
    }

    /**
     * Gets the current state of the swerve drivetrain.
     *
     * @return Current state of the drivetrain
     */
    public SwerveDriveState getState() {
        try {
            m_stateLock.readLock().lock();

            var state = new SwerveDriveState();
            return state;
        } finally {
            m_stateLock.readLock().unlock();
        }
    }

    /**
     * Adds a vision measurement to the Kalman Filter. This will correct the
     * odometry pose estimate
     * while still accounting for measurement noise.
     *
     * <p>
     * This method can be called as infrequently as you want, as long as you are
     * calling {@link
     * SwerveDrivePoseEstimator#update} every loop.
     *
     * <p>
     * To promote stability of the pose estimate and make it robust to bad vision
     * data, we
     * recommend only adding vision measurements that are already within one meter
     * or so of the
     * current pose estimate.
     *
     * <p>
     * Note that the vision measurement standard deviations passed into this method
     * will continue
     * to apply to future measurements until a subsequent call to {@link
     * SwerveDrivePoseEstimator#setVisionMeasurementStdDevs(Matrix)} or this method.
     *
     * @param visionRobotPoseMeters    The pose of the robot as measured by the
     *                                 vision camera.
     * @param timestampSeconds         The timestamp of the vision measurement in
     *                                 seconds. Note that if you
     *                                 don't use your own time source by calling
     *                                 {@link
     *                                 SwerveDrivePoseEstimator#updateWithTime(double,Rotation2d,SwerveModulePosition[])},
     *                                 then
     *                                 you must use a timestamp with an epoch since
     *                                 FPGA startup (i.e., the epoch of this
     *                                 timestamp is the same epoch as
     *                                 {@link edu.wpi.first.wpilibj.Timer#getFPGATimestamp()}).
     *                                 This means that you should use
     *                                 {@link edu.wpi.first.wpilibj.Timer#getFPGATimestamp()}
     *                                 as
     *                                 your time source in this case.
     * @param visionMeasurementStdDevs Standard deviations of the vision pose
     *                                 measurement (x position
     *                                 in meters, y position in meters, and heading
     *                                 in radians). Increase these numbers to trust
     *                                 the vision pose measurement less.
     */
    public void addVisionMeasurement(
            Pose2d visionRobotPoseMeters,
            double timestampSeconds,
            Matrix<N3, N1> visionMeasurementStdDevs) {
        try {
            m_stateLock.writeLock().lock();
            m_odometry.addVisionMeasurement(visionRobotPoseMeters, timestampSeconds, visionMeasurementStdDevs);
        } finally {
            m_stateLock.writeLock().unlock();
        }
    }

    /**
     * Adds a vision measurement to the Kalman Filter. This will correct the
     * odometry pose estimate
     * while still accounting for measurement noise.
     *
     * <p>
     * This method can be called as infrequently as you want, as long as you are
     * calling {@link
     * SwerveDrivePoseEstimator#update} every loop.
     *
     * <p>
     * To promote stability of the pose estimate and make it robust to bad vision
     * data, we
     * recommend only adding vision measurements that are already within one meter
     * or so of the
     * current pose estimate.
     *
     * @param visionRobotPoseMeters The pose of the robot as measured by the vision
     *                              camera.
     * @param timestampSeconds      The timestamp of the vision measurement in
     *                              seconds. Note that if you
     *                              don't use your own time source by calling {@link
     *                              SwerveDrivePoseEstimator#updateWithTime(double,Rotation2d,SwerveModulePosition[])}
     *                              then you
     *                              must use a timestamp with an epoch since FPGA
     *                              startup (i.e., the epoch of this timestamp is
     *                              the same epoch as
     *                              {@link edu.wpi.first.wpilibj.Timer#getFPGATimestamp()}.)
     *                              This means that
     *                              you should use
     *                              {@link edu.wpi.first.wpilibj.Timer#getFPGATimestamp()}
     *                              as your time source
     *                              or sync the epochs.
     */
    public void addVisionMeasurement(Pose2d visionRobotPoseMeters, double timestampSeconds) {
        try {
            m_stateLock.writeLock().lock();
            m_odometry.addVisionMeasurement(visionRobotPoseMeters, timestampSeconds);
        } finally {
            m_stateLock.writeLock().unlock();
        }
    }

    /**
     * Sets the pose estimator's trust of global measurements. This might be used to
     * change trust in
     * vision measurements after the autonomous period, or to change trust as
     * distance to a vision
     * target increases.
     *
     * @param visionMeasurementStdDevs Standard deviations of the vision
     *                                 measurements. Increase these
     *                                 numbers to trust global measurements from
     *                                 vision less. This matrix is in the form [x,
     *                                 y,
     *                                 theta]ᵀ, with units in meters and radians.
     */
    public void setVisionMeasurementStdDevs(Matrix<N3, N1> visionMeasurementStdDevs) {
        try {
            m_stateLock.writeLock().lock();
            m_odometry.setVisionMeasurementStdDevs(visionMeasurementStdDevs);
        } finally {
            m_stateLock.writeLock().unlock();
        }
    }

    /**
     * Updates all the simulation state variables for this
     * drivetrain class. User provides the update variables for the simulation.
     *
     * @param dtSeconds time since last update call
     * @param supplyVoltage voltage as seen at the motor controllers
     */
    public void updateSimState(double dtSeconds, double supplyVoltage) {
        m_simDrive.update(dtSeconds, supplyVoltage, Modules);
    }


    /**
     * Register the specified lambda to be executed whenever our SwerveDriveState function
     * is updated in our odometry thread.
     * <p>
     * It is imperative that this function is cheap, as it will be executed along with
     * the odometry call, and if this takes a long time, it may negatively impact
     * the odometry of this stack.
     * <p>
     * This can also be used for logging data if the function performs logging instead of telemetry
     *
     * @param telemetryFunction Function to call for telemetry or logging
     */
    public void registerTelemetry(Consumer<SwerveDriveState> telemetryFunction) {
        m_telemetryFunction = telemetryFunction;
    }
}
