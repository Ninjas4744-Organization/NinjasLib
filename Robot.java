package frc.lib.NinjasLib;

import com.pathplanner.lib.config.PIDConstants;
import com.pathplanner.lib.config.RobotConfig;
import com.pathplanner.lib.controllers.PPHolonomicDriveController;
import com.pathplanner.lib.controllers.PathFollowingController;
import com.pathplanner.lib.path.PathConstraints;
import edu.wpi.first.apriltag.AprilTagFieldLayout;
import edu.wpi.first.apriltag.AprilTagFields;
import edu.wpi.first.math.Pair;
import edu.wpi.first.math.geometry.Rotation3d;
import edu.wpi.first.math.geometry.Transform3d;
import edu.wpi.first.math.geometry.Translation2d;
import edu.wpi.first.math.kinematics.ChassisSpeeds;
import edu.wpi.first.math.kinematics.SwerveDriveKinematics;
import edu.wpi.first.wpilibj.DriverStation;
import edu.wpi.first.wpilibj.PowerDistribution;
import edu.wpi.first.wpilibj.PowerDistribution.ModuleType;
import edu.wpi.first.wpilibj2.command.CommandScheduler;
import edu.wpi.first.wpilibj2.command.Commands;
import edu.wpi.first.wpilibj2.command.button.CommandPS5Controller;
import frc.lib.NinjasLib.Controllers.NinjasSimulatedController;
import frc.lib.NinjasLib.Controllers.NinjasTalonFXController;
import frc.lib.NinjasLib.DataClasses.*;
import frc.lib.NinjasLib.Subsystems.StateMachineMotoredSubsystem;
import frc.lib.NinjasLib.Swerve.SwerveIO;
import frc.lib.NinjasLib.Vision.VisionIO;
import org.json.simple.parser.ParseException;
import org.littletonrobotics.junction.LogFileUtil;
import org.littletonrobotics.junction.LoggedRobot;
import org.littletonrobotics.junction.Logger;
import org.littletonrobotics.junction.networktables.NT4Publisher;
import org.littletonrobotics.junction.wpilog.WPILOGReader;
import org.littletonrobotics.junction.wpilog.WPILOGWriter;

import java.io.IOException;
import java.util.List;
import java.util.Map;

public class Robot extends LoggedRobot {
//  NinjasController _shooter;
//  CommandPS5Controller _controller;

    public static AprilTagFieldLayout kBlueFieldLayout;
    public static AprilTagFieldLayout kRedFieldLayout;

    static {
        try{
            kBlueFieldLayout = AprilTagFieldLayout.loadFromResource(AprilTagFields.k2024Crescendo.m_resourceFile);
            kBlueFieldLayout.setOrigin(AprilTagFieldLayout.OriginPosition.kBlueAllianceWallRightSide);

            kRedFieldLayout = AprilTagFieldLayout.loadFromResource(AprilTagFields.k2024Crescendo.m_resourceFile);
            kRedFieldLayout.setOrigin(AprilTagFieldLayout.OriginPosition.kRedAllianceWallRightSide);
        }catch(IOException e){
            throw new RuntimeException("Unable to load field layout");
        }
    }

    public AprilTagFieldLayout getFieldLayout(List<Integer> ignoredTags) {
        AprilTagFieldLayout layout;

        layout = RobotStateIO.getAlliance() == DriverStation.Alliance.Blue
          ? kBlueFieldLayout
          : kRedFieldLayout;

        if (!ignoredTags.isEmpty()) layout.getTags().removeIf(tag -> ignoredTags.contains(tag.ID));

        return layout;
    }

    public AprilTagFieldLayout getFieldLayout() {
        return getFieldLayout(List.of());
    }

    public enum st{
        hey
    }

     public class SwerveConstants {
         public static final frc.lib.NinjasLib.DataClasses.SwerveConstants kSwerveConstants = new frc.lib.NinjasLib.DataClasses.SwerveConstants();
         static {
             kSwerveConstants.openLoop = true;
             kSwerveConstants.trackWidth = 0.62;
             kSwerveConstants.wheelBase = 0.62;
             kSwerveConstants.kinematics = new SwerveDriveKinematics(
                     new Translation2d(kSwerveConstants.wheelBase / 2.0, kSwerveConstants.trackWidth / 2.0),
                     new Translation2d(kSwerveConstants.wheelBase / 2.0, -kSwerveConstants.trackWidth / 2.0),
                     new Translation2d(-kSwerveConstants.wheelBase / 2.0, kSwerveConstants.trackWidth / 2.0),
                     new Translation2d(-kSwerveConstants.wheelBase / 2.0, -kSwerveConstants.trackWidth / 2.0)
             );

             kSwerveConstants.maxSpeed = 5;
             kSwerveConstants.maxAngularVelocity = 10.7;
             kSwerveConstants.speedLimit = 5;
             kSwerveConstants.rotationSpeedLimit = 10.7;
             kSwerveConstants.accelerationLimit = 10;
             kSwerveConstants.rotationAccelerationLimit = 54;

             kSwerveConstants.enableLogging = true;
             kSwerveConstants.moduleConstants = new SwerveModuleConstants[4];

             for (int i = 0; i < 4; i++) {
                 kSwerveConstants.moduleConstants[i] = new SwerveModuleConstants<>(i,
                         new MainControllerConstants(),
                         new MainControllerConstants(),
                         kSwerveConstants.maxSpeed,
                         6 + i,
                         NinjasTalonFXController.class,
                         NinjasTalonFXController.class,
                         true,
                         false,
                         0);

                 kSwerveConstants.moduleConstants[i].driveMotorConstants.main.id = 10 + i * 2;
                 kSwerveConstants.moduleConstants[i].driveMotorConstants.currentLimit = 68;
                 kSwerveConstants.moduleConstants[i].driveMotorConstants.encoderConversionFactor = 0.056267331109070916;
                 kSwerveConstants.moduleConstants[i].driveMotorConstants.subsystemName = "Swerve Module " + i + " Drive Motor";
                 kSwerveConstants.moduleConstants[i].driveMotorConstants.enableLogging = true;
                 kSwerveConstants.moduleConstants[i].driveMotorConstants.controlConstants = ControlConstants.createTorqueCurrent(5, 0.1);

                 kSwerveConstants.moduleConstants[i].angleMotorConstants.main.id = 11 + i * 2;
                 kSwerveConstants.moduleConstants[i].angleMotorConstants.main.inverted = false;
                 kSwerveConstants.moduleConstants[i].angleMotorConstants.currentLimit = 50;
                 kSwerveConstants.moduleConstants[i].angleMotorConstants.encoderConversionFactor = 19.2;
                 kSwerveConstants.moduleConstants[i].angleMotorConstants.subsystemName = "Swerve Module " + i + " Angle Motor";
                 kSwerveConstants.moduleConstants[i].angleMotorConstants.enableLogging = true;
                 kSwerveConstants.moduleConstants[i].angleMotorConstants.controlConstants = ControlConstants.createPID(4 / 19.2, 0, 0, 0);
             }

             kSwerveConstants.moduleConstants[0].CANCoderOffset = -0.290039;
             kSwerveConstants.moduleConstants[1].CANCoderOffset = 0.226562;
             kSwerveConstants.moduleConstants[2].CANCoderOffset = 0.235596;
             kSwerveConstants.moduleConstants[3].CANCoderOffset = 0.274170;
         }

         public static final SwerveControllerConstants kSwerveControllerConstants = new SwerveControllerConstants();
         static {
             kSwerveControllerConstants.swerveConstants = kSwerveConstants;
             kSwerveControllerConstants.drivePIDConstants = ControlConstants.createPID(1, 0, 0, 0);
             kSwerveControllerConstants.rotationPIDConstants = ControlConstants.createPID(0.057, 0.09, 0.003, 10);
             kSwerveControllerConstants.axisLockPIDConstants = ControlConstants.createPID(0.14, 0, 0, 0);
             kSwerveControllerConstants.driveAssistThreshold = 2;
             kSwerveControllerConstants.pathConstraints = new PathConstraints(5, 10, 8, 16);

             try {
                kSwerveControllerConstants.robotConfig = RobotConfig.fromGUISettings();
            } catch (IOException e) {
                // TODO Auto-generated catch block
                e.printStackTrace();
            } catch (ParseException e) {
                // TODO Auto-generated catch block
                e.printStackTrace();
            }//new RobotConfig(50, 20, new ModuleConfig(0.04, 5, 1, DCMotor.getKrakenX60(1), 60, 4), 0.7);
             
            kSwerveControllerConstants.rotationPIDContinuousConnections = Pair.of(-180.0, 180.0);
         }

         public static final PathFollowingController kPathFollowingController =
           new PPHolonomicDriveController(
             new PIDConstants(kSwerveControllerConstants.drivePIDConstants.P, kSwerveControllerConstants.drivePIDConstants.I, kSwerveControllerConstants.drivePIDConstants.D),
             new PIDConstants(kSwerveControllerConstants.rotationPIDConstants.P, kSwerveControllerConstants.rotationPIDConstants.I, kSwerveControllerConstants.rotationPIDConstants.D)
           );
     }

     public class TestSub extends StateMachineMotoredSubsystem<st>{
         public TestSub(boolean paused) {
             super(paused);
         }

//         public TestSub getInstance

         @Override
         protected void setController() {
            MainControllerConstants c = new MainControllerConstants();
            c.subsystemName = "Test";
            c.controlConstants = ControlConstants.createPID(1, 0, 0, 0);
            c.positionGoalTolerance = 5;
            _controller = new NinjasTalonFXController(c);
         }

         @Override
         protected void setSimulationController() {
             SimulatedControllerConstants c = new SimulatedControllerConstants();
             c.mainControllerConstants.subsystemName = "Test";
             c.mainControllerConstants.controlConstants = ControlConstants.createPID(1, 0, 0, 0);
             c.mainControllerConstants.positionGoalTolerance = 5;
             c.motorType = SimulatedControllerConstants.MotorType.KRAKEN;
            _simulatedController = new NinjasSimulatedController(c);
         }

         @Override
         protected void resetSubsystemO() {
            controller().setPosition(0);
         }

         @Override
         protected boolean isResettedO() {
             return controller().isHomed();
         }

         @Override
         protected void setFunctionMaps() {

         }
     }

     public class RobotState extends RobotStateWithSwerve<st> {
        public RobotState(){
            _robotState = st.hey;
        }
     }

    NinjasSimulatedController shooterAngle;
    NinjasSimulatedController shooter;
    NinjasTalonFXController _yes;
    CommandPS5Controller _controller = new CommandPS5Controller(0);
    public Robot() {
        boolean replayLastGame = false;
        if (!(replayLastGame && isSimulation())) {
//            Logger.addDataReceiver(new WPILOGWriter()); // Log to a USB stick ("/U/logs")
            Logger.addDataReceiver(new NT4Publisher()); // Publish data to NetworkTables
            new PowerDistribution(1, ModuleType.kRev); // Enables power distribution logging
        } else {
            setUseTiming(false); // Run as fast as possible
            String logPath = LogFileUtil.findReplayLog(); // Pull the replay log from AdvantageScope (or prompt the user)
            Logger.setReplaySource(new WPILOGReader(logPath)); // Read replay log
            Logger.addDataReceiver(new WPILOGWriter(LogFileUtil.addPathSuffix(logPath, "_sim"))); // Save outputs to a new log
        }

        Logger.start(); // Start logging! No more data receivers, replay sources, or metadata values may be added.
        
//    MainControllerConstants c = new MainControllerConstants();
//    c.main.id = 30;
//    c.controlConstants = ControlConstants.createTorqueCurrent(7.5, 0);
//    _shooter = new NinjasTalonFXController(c);
//
//    _controller.cross().whileTrue(Commands.startEnd(() -> _shooter.setVelocity(100), () -> _shooter.stop()));

//        SwerveIO.setConstants(SwerveConstants.kSwerveConstants);
//        RobotStateWithSwerve.setInstance(new RobotState(), SwerveConstants.kSwerveConstants.kinematics, false, (o) -> 0, 45);
//        SwerveController.setConstants(SwerveConstants.kSwerveControllerConstants, SwerveIO.getInstance());

//        VisionConstants kVisionConstants = new VisionConstants();
//        kVisionConstants.cameras = Map.of(
//          "Front", new Transform3d(0.28 - 0.11 - 0.2, 0.105, -0.055, new Rotation3d(0, 30, 0)));
//
//        kVisionConstants.maxAmbiguity = 0.2;
//        kVisionConstants.maxDistance = 4;
//        kVisionConstants.fieldLayoutGetter = this::getFieldLayout;
//
//        VisionIO.setConstants(kVisionConstants);

//        SimulatedControllerConstants c = new SimulatedControllerConstants();
//        c.mainControllerConstants.subsystemName = "ShooterAngle";
//        c.mainControllerConstants.controlConstants = ControlConstants.createPID(0.15, 0, 0, 0);
//        c.mainControllerConstants.positionGoalTolerance = 0.5;
//        c.mainControllerConstants.encoderConversionFactor = 1.0 / 300.0 * 360.0;
//        c.mainControllerConstants.encoderHomePosition = 31;
//        c.motorType = SimulatedControllerConstants.MotorType.KRAKEN_PRO;
//
//        shooterAngle = new NinjasSimulatedController(c);
//
//        _controller.cross().toggleOnTrue(Commands.startEnd(() -> shooterAngle.setPosition(70), () -> shooterAngle.setPosition(31)));
//        _controller.square().toggleOnTrue(Commands.startEnd(() -> shooterAngle.setPercent(1), () -> shooterAngle.setPercent(0)));

//        SimulatedControllerConstants c = new SimulatedControllerConstants();
//        c.mainControllerConstants.subsystemName = "Shooter";
//        c.mainControllerConstants.controlConstants = ControlConstants.createTorqueCurrent(3, 0.185);
//        c.mainControllerConstants.velocityGoalTolerance = 600;
//        c.mainControllerConstants.encoderConversionFactor = 60;
//        c.motorType = SimulatedControllerConstants.MotorType.FALCON_PRO;
//        shooter = new NinjasSimulatedController(c);
//
//        _controller.cross().toggleOnTrue(Commands.startEnd(() -> shooter.setVelocity(6000), () -> shooter.setVelocity(0)));

//        SwerveController.getInstance().setState(SwerveDemand.SwerveState.DRIVE_ASSIST);
//        SwerveController.getInstance().Demand.targetPose = new Pose2d(3, 8, Rotation2d.fromDegrees(90));
//        NetworkTableInstance.getDefault().getDoubleArrayTopic("Target").getEntry(new double[]{ 3, 8, Units.degreesToRadians(90) }).set(new double[]{ 3, 8, Units.degreesToRadians(90) });
//        _controller.square().onTrue(Commands.runOnce(() -> {
//            SwerveController.getInstance().setState(SwerveDemand.SwerveState.DEFAULT);
//            SwerveController.getInstance().setState(SwerveDemand.SwerveState.DRIVE_ASSIST);
//        }));

//        List<Waypoint> waypoints = PathPlannerPath.waypointsFromPoses(
//                new Pose2d(0, 0, Rotation2d.kZero),
//                new Pose2d(0.15, 0, Rotation2d.kZero)
//        );
//
//        PathPlannerPath path = new PathPlannerPath(
//                waypoints,
//                SwerveConstants.kSwerveControllerConstants.pathConstraints,
//                null,
//                new GoalEndState(0.01, Rotation2d.kZero));
//
//        PathPlannerTrajectory traj = new PathPlannerTrajectory(
//                path,
//                SwerveIO.getInstance().getChassisSpeeds(false),
//                Rotation2d.kZero,
//                SwerveConstants.kSwerveControllerConstants.robotConfig);

        SwerveIO.setConstants(SwerveConstants.kSwerveConstants);
        RobotStateWithSwerve.setInstance(new RobotState(), SwerveConstants.kSwerveConstants.kinematics, false, (o) -> new double[0], 45);
        StateMachineIO.setInstance(new StateMachineIO<st>(false) {
            @Override
            protected boolean canChangeRobotState(st currentState, st wantedState) {
                return true;
            }

            @Override
            protected void setCommandMap() {
                addCommand(st.hey,Commands.runOnce(() -> {}));
            }
//            @Override
//            protected void setEndConditionMap() {
////                addEndCondition(st.hey, new StateEndCondition<>(() -> true, st.hey));
////                addEndCondition(st.hey, new StateEndCondition<>(() -> true, st.hey));
//            }

            @Override
            protected void setFunctionMaps() {

            }
        });

        VisionConstants v = new VisionConstants();
        v.cameras = Map.of("", Pair.of(new Transform3d(0, 0, 0, Rotation3d.kZero), VisionConstants.CameraType.Limelight));
        v.fieldLayoutGetter = this::getFieldLayout;
        v.maxAmbiguity = 6;
        v.maxDistance = 6;
        VisionIO.setConstants(v);

        MainControllerConstants c = new MainControllerConstants();
        c.main.id = 30;
        c.controlConstants = ControlConstants.createPID(1, 0, 0, 0);
        c.encoderConversionFactor = 1 / 10.0;
        c.positionGoalTolerance = 0.1;
        c.subsystemName = "yes";
        c.currentLimit = 40;
        _yes = new NinjasTalonFXController(c);
        _yes.resetEncoder();

        _controller.cross().onTrue(Commands.runOnce(() -> _yes.setPosition(1)));
        _controller.circle().onTrue(Commands.runOnce(() -> _yes.setPosition(0)));
        _controller.square().onTrue(Commands.runOnce(() -> _yes.setEncoder(0.5)));
    }

    @Override
    public void robotPeriodic() {
        CommandScheduler.getInstance().run();
        _yes.periodic();
//        System.out.println(Arrays.toString(LimelightHelpers.getBotPose("")));
//        SmartDashboard.putString("Smegma", Arrays.toString(LimelightHelpers.getBotPose("")));
//        SwerveController.getInstance().periodic();
//        shooterAngle.periodic();
//    _shooter.periodic();
//    System.out.println(VisionIO.getInstance().getVisionEstimations()[0].closestTagDist);
//        for(VisionOutput o : VisionIO.getInstance().getVisionEstimations()){
//            RobotStateWithSwerve.getInstance().updateRobotPose(o);
//        }
//
//        SwerveController.getInstance().periodic();
//        SwerveIO.getInstance().periodic();
//
//        SmartDashboard.putString("Swerve State", SwerveController.getInstance().getState().toString());
    }

    @Override
    public void disabledInit() {}

    @Override
    public void disabledPeriodic() {}

    @Override
    public void disabledExit() {}

    @Override
    public void autonomousInit() {
//        Commands.run(() -> {
//            Translation2d pid = SwerveController.getInstance().pidTo(new Translation2d(3, 6));
//            SmartDashboard.putNumber("pidx", pid.getX());
//            SmartDashboard.putNumber("pidy", pid.getY());
//            SwerveController.getInstance().setState(SwerveDemand.SwerveState.VELOCITY);
//            SwerveController.getInstance()._demand.fieldRelative = true;
//            SwerveController.getInstance()._demand.velocity = new ChassisSpeeds(pid.getX(), pid.getY(), 0);
//        }).repeatedly().schedule();
    }

    @Override
    public void autonomousPeriodic() {}

    @Override
    public void autonomousExit() {}

    @Override
    public void teleopInit() {}

    @Override
    public void teleopPeriodic() {
        SwerveIO.getInstance().drive(new ChassisSpeeds(-_controller.getLeftY() * 5, -_controller.getLeftX() * 5, -_controller.getRightX() * 11), false);
//        SwerveController.getInstance().Demand.driverInput = new ChassisSpeeds(-MathUtil.applyDeadband(_controller.getLeftY(), 0.1) * 0.4, -MathUtil.applyDeadband(_controller.getLeftX(), 0.1) * 0.4, -MathUtil.applyDeadband(_controller.getRightX(), 0.1) * 0.25);
//        SwerveController.getInstance().setState(SwerveDemand.SwerveState.DEFAULT);
        SwerveIO.getInstance().periodic();
//        shooter.periodic();

//        SwerveController.getInstance().Demand.driverInput = new ChassisSpeeds(-_controller.getLeftY(), -_controller.getLeftX(), -_controller.getRightX());
    }

    @Override
    public void teleopExit() {}

    @Override
    public void testInit() {
        CommandScheduler.getInstance().cancelAll();
    }

    @Override
    public void testPeriodic() {}

    @Override
    public void testExit() {}
}
