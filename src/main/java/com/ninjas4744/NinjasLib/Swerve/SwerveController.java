package com.ninjas4744.NinjasLib.Swerve;

import com.ninjas4744.NinjasLib.DataClasses.SwerveControllerConstants;
import com.ninjas4744.NinjasLib.DataClasses.SwerveDemand;
import com.ninjas4744.NinjasLib.RobotStateWithSwerve;
import com.pathplanner.lib.path.GoalEndState;
import com.pathplanner.lib.path.IdealStartingState;
import com.pathplanner.lib.path.PathPlannerPath;
import com.pathplanner.lib.path.Waypoint;
import com.pathplanner.lib.trajectory.PathPlannerTrajectory;
import com.pathplanner.lib.trajectory.PathPlannerTrajectoryState;
import edu.wpi.first.math.controller.PIDController;
import edu.wpi.first.math.geometry.Pose2d;
import edu.wpi.first.math.geometry.Rotation2d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.geometry.Translation3d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.wpilibj.Timer;
import edu.wpi.first.wpilibj.shuffleboard.Shuffleboard;
import org.littletonrobotics.junction.Logger;

import java.util.List;

public class SwerveController {
    private final PIDController _anglePID;
    private final PIDController _xPID;
    private final PIDController _yPID;
    private final PIDController _axisPID;
    // private final Timer pathfindingTimer = new Timer();
    // private PathPlannerTrajectory pathfindingCurrentTraj = null;
    private final SwerveControllerConstants _constants;
    private final SwerveIO _swerve;

    private final Timer _driveAssistTimer = new Timer();
    private Pose2d _driveAssistTargetPose;
    private PathPlannerTrajectory _driveAssistTrajectory;

    public final SwerveDemand Demand;
    private SwerveDemand.SwerveState _state;
    private SwerveDemand.SwerveState _previousState;

    private static SwerveController _instance = null;

    public static void setConstants(SwerveControllerConstants constants, SwerveIO swerve) {
        _instance = new SwerveController(constants, swerve);
    }

    public static SwerveController getInstance() {
        if(_instance == null)
            throw new RuntimeException("SwerveController constants not given. Initialize SwerveController by setConstants(SwerveControllerConstants, SwerveIO) first.");
        return _instance;
    }

    private SwerveController(SwerveControllerConstants constants, SwerveIO swerve) {
        _constants = constants;
        _swerve = swerve;

        _state = SwerveDemand.SwerveState.DEFAULT;
        _previousState = SwerveDemand.SwerveState.DEFAULT;
        Demand = new SwerveDemand();

        _anglePID = new PIDController(
            constants.rotationPIDConstants.P,
            constants.rotationPIDConstants.I,
            constants.rotationPIDConstants.D
        );
        _anglePID.setIZone(constants.rotationPIDConstants.IZone);
        _anglePID.enableContinuousInput(constants.rotationPIDContinuousConnections.getFirst(), constants.rotationPIDContinuousConnections.getSecond());

        _axisPID = new PIDController(
            constants.axisLockPIDConstants.P,
            constants.axisLockPIDConstants.I,
            constants.axisLockPIDConstants.D);
        _axisPID.setIZone(constants.axisLockPIDConstants.IZone);

        _xPID = new PIDController(
                constants.drivePIDConstants.P,
                constants.drivePIDConstants.I,
                constants.drivePIDConstants.D);
        _xPID.setIZone(constants.drivePIDConstants.IZone);

        _yPID = new PIDController(
                constants.drivePIDConstants.P,
                constants.drivePIDConstants.I,
                constants.drivePIDConstants.D);
        _yPID.setIZone(constants.drivePIDConstants.IZone);

//        if(!constants.swerveConstants.createShuffleBoard)
//            return;

//        Shuffleboard.getTab("Swerve").addBoolean("Drive Assist Finished", this::isDriveAssistFinished);
//        Shuffleboard.getTab("Swerve").addNumber("Driver Input X", () -> Demand.driverInput.vxMetersPerSecond);
//        Shuffleboard.getTab("Swerve").addNumber("Driver Input Y", () -> Demand.driverInput.vyMetersPerSecond);
//        Shuffleboard.getTab("Swerve").addNumber("Driver Input Omega", () -> Demand.driverInput.omegaRadiansPerSecond);
//        Shuffleboard.getTab("Swerve").addString("State", () -> _state.toString());
//        Shuffleboard.getTab("Swerve").addString("Previous State", () -> _previousState.toString());
    }

    /**
     * Makes the swerve use PID to look at the given angle
     *
     * @param angle - the angle to look at
     * @param roundToAngle - the angle jumps to round to, for example 45 degrees will make it round
     *     the given angle to the nearest 0, 45, 90, 135... it rounds the angle only if the rounded
     *     angle is close enough to the given angle so for example if the given angle is 28 and the
     *     rounded angle is 45 it won't round. if you write 1 as the roundToAngle there will be no
     *     rounding, DON'T USE 0 (division by zero error)
     */
    public double lookAt(double angle, double roundToAngle) {
        double roundedAngle = Math.round(angle / roundToAngle) * roundToAngle;
        angle = Math.abs(roundedAngle - angle) <= roundToAngle / 3 ? roundedAngle : angle;

        return _anglePID.calculate(RobotStateWithSwerve.getInstance().getGyroYaw().getDegrees(), angle);
    }

    /**
     * Makes the swerve use PID to look according to the given direction
     *
     * @param direction - the direction vector to look
     * @param roundToAngle - the angle jumps to round to, for example 45 degrees will make it round
     *     the given angle (calculated from direction) to the nearest 0, 45, 90, 135... it rounds the
     *     angle only if the rounded angle is close enough to the given angle so for example if the
     *     given angle is 28 and the rounded angle is 45 it won't round. if you write 1 as the
     *     roundToAngle there will be no rounding, DON'T USE 0 (division by zero error)
     */
    public double lookAt(Translation2d direction, double roundToAngle) {
        if (!(direction.getX() == 0 && direction.getY() == 0))
            return lookAt(direction.getAngle().getDegrees(), roundToAngle);

        return 0;
    }

    public double lookAtTarget(Pose2d target, Rotation2d offset) {
        Translation2d lookAtTranslation = RobotStateWithSwerve.getInstance().getDistanceTo(target).rotateBy(offset);
        return lookAt(lookAtTranslation, 1);
    }

    public Translation2d pidTo(Translation2d target) {
        return new Translation2d(
            _xPID.calculate(RobotStateWithSwerve.getInstance().getRobotPose().getX(), target.getX()),
            _yPID.calculate(RobotStateWithSwerve.getInstance().getRobotPose().getY(), target.getY()));
    }

    private ChassisSpeeds pathfindTo(Pose2d pose, ChassisSpeeds driverInput) {
        return new ChassisSpeeds(0, 0, 0);
//        Pathfinding.setGoalPosition(pose.getTranslation());
//        Pathfinding.setStartPosition(RobotStateWithSwerve.getInstance().getRobotPose().getTranslation());
//
//        PathPlannerPath path = Pathfinding.getCurrentPath(
//            constants.kConstraints, new GoalEndState(0, pose.getRotation()));
//        if (path == null) {
//            System.out.println("No path available");
//            return;
//        }
//        PathPlannerTrajectory trajectory = new PathPlannerTrajectory(
//            path, getChassisSpeeds(true), RobotStateWithSwerve.getInstance().getRobotPose().getRotation());
//        if (pathfindingCurrentTraj == null
//            || pathfindingCurrentTraj.getTotalTimeSeconds() != trajectory.getTotalTimeSeconds()) {
//            System.out.println("New path available");
//            pathfindingCurrentTraj = trajectory;
//            pathfindingTimer.restart();
//        }
//
//        double feedforwardX = trajectory.sample(pathfindingTimer.get()).velocityMps
//            * trajectory.sample(pathfindingTimer.get()).heading.getCos();
//        double feedforwardY = trajectory.sample(pathfindingTimer.get()).velocityMps
//            * trajectory.sample(pathfindingTimer.get()).heading.getSin();
//
//        Translation2d pid = pidTo(trajectory.sample(pathfindingTimer.get()).positionMeters);
//
//        return new ChassisSpeeds(
//                1 * feedforwardX + 0 * pid.getX() + driverInput.vxMetersPerSecond,
//                1 * feedforwardY + 0 * pid.getY() + driverInput.vyMetersPerSecond,
//                driverInput.omegaRadiansPerSecond);
    }

    /**
     * Makes the swerve be locked to an axis with a pid that ensures that. The driver input let the driver move along the axis.
     *
     * @param angle          angle of the axis
     * @param point           a point on the axis in meters
     * @param driverInput    the driver controller input
     * @param isXDriverInput whether to let the driver drive along the axis by the x of the joystick input or the y
     */
    private ChassisSpeeds lockAxis(Rotation2d angle, Pose2d point, ChassisSpeeds driverInput, boolean isXDriverInput, boolean invertDriverInput) {
        Translation2d axis = new Translation2d(-1, angle);
        Translation2d perpendicularAxis = axis.rotateBy(Rotation2d.fromDegrees(90));
        Translation2d robotPose = RobotStateWithSwerve.getInstance().getRobotPose().getTranslation();

        double a = -axis.getY();
        double b = axis.getX();
//        double c = -phase * Math.sqrt(a * a + b * b);
        double c = - a * point.getX() - b * point.getY();
        double error = -(a * robotPose.getX() + b * robotPose.getY() + c) / Math.sqrt(a * a + b * b);
        Translation2d pid = perpendicularAxis.times(_axisPID.calculate(-error));

        Translation2d driver = axis.times(
                isXDriverInput
                ? (!invertDriverInput ? -driverInput.vyMetersPerSecond : driverInput.vyMetersPerSecond)
                : (!invertDriverInput ? -driverInput.vxMetersPerSecond : driverInput.vxMetersPerSecond)
        );

        return new ChassisSpeeds(
                driver.getX() + pid.getX(),
                driver.getY() + pid.getY(),
                driverInput.omegaRadiansPerSecond);
    }

    /**
     * follows path to given target
     * @param targetPose - given target
     * @return Calculated chassis speeds, field relative
     */
    public ChassisSpeeds driveAssist(Pose2d targetPose) {
        if (targetPose != _driveAssistTargetPose) {
            _driveAssistTargetPose = targetPose;
            startingDriveAssist(_driveAssistTargetPose);
        }

        return calculateDriveAssist();
    }

    private ChassisSpeeds calculateDriveAssist() {
        PathPlannerTrajectoryState desiredState = _driveAssistTrajectory.sample(_driveAssistTimer.get());

//        Rotation2d heading = desiredState.;
//        double xFeedforward = desiredState.linearVelocity * heading.getCos();
//        double yFeedforward = desiredState.linearVelocity * heading.getSin();

        double thetaFeedback = lookAt(desiredState.pose.getRotation().getDegrees(), 1);
        Translation2d feedback = pidTo(desiredState.pose.getTranslation());

        return new ChassisSpeeds(desiredState.fieldSpeeds.vxMetersPerSecond + feedback.getX(), desiredState.fieldSpeeds.vyMetersPerSecond + feedback.getY(), thetaFeedback);
    }

    private void startingDriveAssist(Pose2d targetPose) {
        Rotation2d dir = RobotStateWithSwerve.getInstance().getDistanceTo(targetPose).getAngle();
        List<Waypoint> waypoints = PathPlannerPath.waypointsFromPoses(
            new Pose2d(RobotStateWithSwerve.getInstance().getRobotPose().getX(), RobotStateWithSwerve.getInstance().getRobotPose().getY(), dir),
            new Pose2d(targetPose.getX(), targetPose.getY(), dir)
        );

        PathPlannerPath _path = new PathPlannerPath(
            waypoints,
            _constants.pathConstraints,
            null,
            new GoalEndState(0.01, targetPose.getRotation()));

        _driveAssistTrajectory = new PathPlannerTrajectory(
            _path,
            _swerve.getChassisSpeeds(false),
            RobotStateWithSwerve.getInstance().getRobotPose().getRotation(),
            _constants.robotConfig);

        _driveAssistTimer.restart();
    }

    /**
     * @return Whether the path following was finished, will return false if not started
     */
    public boolean isDriveAssistFinished() {
        return _driveAssistTrajectory != null && _driveAssistTimer.get() > _driveAssistTrajectory.getTotalTimeSeconds();
    }

    /**
     * Set the current state of the swerve, so it will work according
     * @param state the wanted state
     */
    public void setState(SwerveDemand.SwerveState state) {
        _previousState = _state;
        _state = state;

        if(state != SwerveDemand.SwerveState.DRIVE_ASSIST){
            _driveAssistTargetPose = null;
            _driveAssistTrajectory = null;
            _driveAssistTimer.stop();
            _driveAssistTimer.reset();
        }
    }

    /**
     * @return the current state of the swerve
     */
    public SwerveDemand.SwerveState getState() {
        return _state;
    }

    /**
     * @return the previous state of the swerve, the state it was before changing it
     */
    public SwerveDemand.SwerveState getPreviousState() {
        return _previousState;
    }

    public void periodic() {
        ChassisSpeeds driverInput = _swerve.fromPercent(Demand.driverInput);

        switch (_state) {
            case DEFAULT:
                _swerve.drive(driverInput, _constants.driverFieldRelative);
                break;

            case DRIVE_ASSIST:
                if((!isDriveAssistFinished() || Demand.targetPose != _driveAssistTargetPose) && RobotStateWithSwerve.getInstance().getDistanceTo(Demand.targetPose).getNorm() <= _constants.driveAssistThreshold)
                    _swerve.drive(driveAssist(Demand.targetPose), true);
                else
                    _swerve.drive(driverInput, _constants.driverFieldRelative);
                break;

            case LOOK_AT_TARGET:
                _swerve.drive(
                    new ChassisSpeeds(
                        driverInput.vxMetersPerSecond,
                        driverInput.vyMetersPerSecond,
                        lookAtTarget(Demand.targetPose, Demand.angleOffset)), true);
                break;

            case PATHFINDING:
                _swerve.drive(pathfindTo(Demand.targetPose, driverInput), true);
                break;

            case VELOCITY:
                _swerve.drive(Demand.velocity, Demand.fieldRelative);
                break;

            case LOCKED_AXIS:
                _swerve.drive(lockAxis(Demand.angle, Demand.point, driverInput, Demand.isXDriverInput, Demand.invertDriverInput), true);
                break;
        }

        _swerve.periodic();

        if(!_constants.swerveConstants.enableLogging)
            return;

        Logger.recordOutput("Swerve/Driver Input", Demand.driverInput);
        Logger.recordOutput("Swerve/Drive Assist Finished", isDriveAssistFinished());
        Logger.recordOutput("Swerve/State", _state.toString());
        Logger.recordOutput("Swerve/Previous State", _previousState.toString());
    }
}
