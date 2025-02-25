package org.firstinspires.ftc.teamcode.tests;

import com.acmerobotics.dashboard.FtcDashboard;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.geometry.Translation2d;
import com.arcrobotics.ftclib.trajectory.Trajectory;
import com.arcrobotics.ftclib.trajectory.TrajectoryConfig;
import com.arcrobotics.ftclib.trajectory.TrajectoryGenerator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.subsystems.nematocyst;
import org.firstinspires.ftc.teamcode.subsystems.swerve.HeadingPIDController;
import org.firstinspires.ftc.teamcode.subsystems.swerve.SwerveDrive;

import java.util.ArrayList;
import java.util.List;

@Autonomous
public class specimenAndGrab extends OpMode {
    SwerveDrive drive;
    nematocyst n;
    FtcDashboard dash;
    Telemetry t2;
    public static double tP = .15;
    public static double tI = 0.001;
    public static double tD = 0.0001;
    public static double smallRP = 0.0375;
    public static double smallRI = 0.0; // was weird at 0.01. 0.005 led to too much error? testing 0.001
    public static double smallRD = 0.0001;
    public static double rP = .275;
    public static double rI = 0.002;
    public static double rD = 0.001;
    double rotPower = 0;
    double xPower = 0;
    double yPower = 0;
//    TrajectorySequence trajSequence;
    PIDController txPID;
    PIDController tyPID;
    HeadingPIDController rotPID;
    PIDController rSmPID;
    Pose2d now = new Pose2d(new Translation2d(0,0), new Rotation2d(0));
    Pose2d trajPose = new Pose2d(new Translation2d(0,0), new Rotation2d(0));
//    TrajectorySequenceBuilder builder;
    ElapsedTime trajTimer;
    ElapsedTime tempBrokenArmTimer;
    Pose2d trajPose2;
    private Pose2d trajPoseGrab;
    ElapsedTime grab2Timer;
    private Pose2d trajPoseGoBack;
    private Pose2d trajPoseB;
    private Pose2d trajPose2B;
    private Pose2d rotatePose;
    private Pose2d maintainPose;

    public enum STATES {
        START,
        FORWARD,
        ARM_UP,
        PULL,
        BACK1,
        RELEASE,
        DRIVEWALL,
        ROTATE,
        ARMDOWN,
        APPROACHWALL,
        GRAB2,
        PULLSPECIMEN,
        GOBACK,
        ARMUP2,
        PARK,
        DONE
    }
    STATES currentState = STATES.START;
    STATES previousState = STATES.START;
    ElapsedTime walltimer;
    ElapsedTime outTimer;
    String[] encoderNames = {
            "fl_encoder",
            "fr_encoder",
            "bl_encoder",
            "br_encoder"
    };
    String[] driveNames = {
            "fl_drive",
            "fr_drive",
            "bl_drive",
            "br_drive"
    };
    String[] angleNames = {
            "fl_angle",
            "fr_angle",
            "bl_angle",
            "br_angle"
    };
    ElapsedTime pullWaiter;
    Trajectory traj1;
    ElapsedTime pauseBegin;
    public Pose2d rotateFTCLibPose(Pose2d odoPose) {
        Pose2d tempPose = new Pose2d(odoPose.getY()*-1,odoPose.getX(), new Rotation2d(odoPose.getHeading()+Math.PI/2));
        return tempPose;
    }
    @Override
    public void init() {
        List<Translation2d> tlist = new ArrayList<>();
        TrajectoryConfig traj1Config = new TrajectoryConfig(24, 24);
        traj1 = TrajectoryGenerator.generateTrajectory(
                new Pose2d(new Translation2d(0,0), new Rotation2d(Math.PI)),
                tlist,
                new Pose2d(new Translation2d(12,0), new Rotation2d(Math.PI)), // idk why, but backwards is positive
                traj1Config
        );

        drive = new SwerveDrive(
                11, 11, 18, 18,
                this, gamepad1, hardwareMap,
                encoderNames, driveNames, angleNames, 0, 0, Math.PI);
//        builder = new TrajectorySequenceBuilder(new Pose2d(12, -66, Math.PI/2),
//                drive.velocityConstraint, drive.accelerationConstraint,
//                drive.maxAngVel, drive.maxAngAccel); // TODO: Maybe bad radians/degrees
//        trajSequence = builder
//                .forward(18)
//                .build();

        n = new nematocyst(this);
        n.init("pivot", "slide", "wrist", "claw");
        dash = FtcDashboard.getInstance();
        t2 = dash.getTelemetry();
        txPID = new PIDController(tP, tI, tD);
        tyPID = new PIDController(tP, tI, tD);
        rotPID = new HeadingPIDController(rP, rI, rD);
        rSmPID = new PIDController(smallRP, smallRI, smallRD);
        trajTimer = new ElapsedTime();
        now = drive.nowPose;
        pullWaiter = new ElapsedTime();
        tempBrokenArmTimer = new ElapsedTime();
        grab2Timer = new ElapsedTime();
        walltimer = new ElapsedTime();
        outTimer = new ElapsedTime();
    }
//    public Pose2d getPoseAtTime(TrajectorySequence trajectorySequence, double time) {
//        double accumulatedTime = 0.0;
//        for (int i = 0; i < trajectorySequence.size(); i++) {
//            TrajectorySegment segment = (TrajectorySegment) trajectorySequence.get(i);
//            double segmentDuration = segment.getDuration();
//            if (accumulatedTime + segmentDuration >= time) {
//                double timeInSegment = time - accumulatedTime;
//                return segment.getPoseAtTime(timeInSegment);
//            }
//            accumulatedTime += segmentDuration;
//        }
//        // If the time exceeds the total duration, return the end pose
//        return trajectorySequence.end();
//    }
    @Override
    public void init_loop () {
        drive.init_loop();
    }
    @Override
    public void start() {
        pauseBegin = new ElapsedTime();
        pauseBegin.reset();
    }
    @Override
    public void loop() {
        switch (currentState) {
            // TODO: DON'T FORGET YOUR BREAK STATEMENTS
            case START:
                if (pauseBegin.seconds() > 1) {
                    currentState = STATES.FORWARD;
                }
                break;
            case FORWARD:
                if (currentState != previousState) {
                    trajPose = new Pose2d(new Translation2d(16.5, 0), new Rotation2d(Math.PI));
                    trajTimer.reset();
                    previousState = STATES.FORWARD;
                } else if (Math.abs(now.getX() - trajPose.getX()) < 1) {
                    currentState = STATES.ARM_UP;
                    drive.loopFC(drive.imu.getRobotYawPitchRollAngles().getYaw(), 0,0,0);
                    xPower = 0;
                    yPower = 0;
                    rotPower = 0;
                }
                now = (drive.nowPose);
                rotPower = rSmPID.calculate(now.getHeading(), trajPose.getHeading());
                xPower =  txPID.calculate(now.getX(), trajPose.getX());
                yPower = tyPID.calculate(now.getY(), trajPose.getY());
                drive.loopFC(drive.imu.getRobotYawPitchRollAngles().getYaw(), yPower, xPower, rotPower);
                break;
            case ARM_UP:
                if (currentState != previousState) {
                    n.goSpecimen(2);
                    n.wristOut();
                    previousState = STATES.ARM_UP;
                } else if ( n.slideMotor.getCurrentPosition() > 2700 &
                        n.targetSlidePosition > 2700
//                        tempBrokenArmTimer.seconds() > 2
                ) {
                    currentState = STATES.PULL;
                }
                n.loop();
                drive.loopFC(drive.imu.getRobotYawPitchRollAngles().getYaw(), 0,0,0);
                break;
            case PULL:
                if (currentState != previousState) {
                    n.wristIn();
                    pullWaiter.reset();
                    previousState = STATES.PULL;
                } else if (pullWaiter.seconds() > 2) {
                    currentState = STATES.BACK1;
                }
                drive.loopFC(drive.imu.getRobotYawPitchRollAngles().getYaw(), 0,0,0);
                break;
            case BACK1:
                if (currentState != previousState) {
                    trajPoseB = new Pose2d(new Translation2d(14.5, 0), new Rotation2d(Math.PI));
                    previousState = STATES.BACK1;
                } else if (Math.abs(now.getX() - trajPoseB.getX()) < 1) {
                    drive.loopFC(drive.imu.getRobotYawPitchRollAngles().getYaw(), 0,0,0);
                    xPower = 0;
                    yPower = 0;
                    rotPower = 0;
                    currentState = STATES.RELEASE;
                }
                now = (drive.nowPose);
                rotPower = 0;
                xPower =  txPID.calculate(now.getX(), trajPoseB.getX());
                yPower = tyPID.calculate(now.getY(), trajPoseB.getY());
                drive.loopFC(drive.imu.getRobotYawPitchRollAngles().getYaw(), yPower, xPower, rotPower);
                break;
            case RELEASE:
                drive.loopFC(drive.imu.getRobotYawPitchRollAngles().getYaw(), 0,0,0);
                n.release();
                n.goUp(0);
                currentState = STATES.DRIVEWALL;
                break;
            case DRIVEWALL:
                if (currentState != previousState) {
                    trajPoseGrab = new Pose2d(new Translation2d(8, -36), new Rotation2d(0));
                    previousState = STATES.DRIVEWALL;
                    outTimer.reset();
                } else if (Math.abs(trajPoseGrab.getX() - now.getX()) < 1 & Math.abs(trajPoseGrab.getY() - now.getY()) < 1) {
                    drive.loopFC(drive.imu.getRobotYawPitchRollAngles().getYaw(), 0,0,0);
                    xPower = 0;
                    yPower = 0;
                    rotPower = 0;
                    currentState = STATES.APPROACHWALL;
                }
                if (outTimer.seconds() > 1.5) {
                    n.getSpecimen();
                }
                now = (drive.nowPose);
                rotPower = rSmPID.calculate(now.getHeading(), trajPoseGrab.getHeading());
                xPower =  txPID.calculate(now.getX(), trajPoseGrab.getX());
                yPower = tyPID.calculate(now.getY(), trajPoseGrab.getY());
                drive.loopFC(drive.imu.getRobotYawPitchRollAngles().getYaw(), yPower, xPower, rotPower);
                break;
            case ROTATE:
                if (currentState != previousState) {
                    previousState = STATES.ROTATE;
                    rotatePose = new Pose2d(new Translation2d(14, 4), new Rotation2d(Math.PI));
                } else if (Math.abs(now.getHeading() - rotatePose.getHeading()) <.1) {
                    currentState = STATES.ARMUP2;
                    drive.loopFC(drive.imu.getRobotYawPitchRollAngles().getYaw(), 0,0,0);
                    xPower = 0;
                    yPower = 0;
                    rotPower = 0;
                }
                now = drive.nowPose;
                rotPower = rotPID.calculate(now.getHeading(), rotatePose.getHeading());
                drive.loopFC(drive.imu.getRobotYawPitchRollAngles().getYaw(), 0,0,rotPower);
                break;
            case APPROACHWALL:
                if (currentState != previousState) {
                    trajPoseGrab = new Pose2d(new Translation2d(2.5, -36), new Rotation2d(0));
                    previousState = STATES.APPROACHWALL;
                } else if (Math.abs(trajPoseGrab.getX() - now.getX()) < 1 & Math.abs(trajPoseGrab.getY() - now.getY()) < 1 & Math.abs(now.getHeading() - 0) < 0.1) {
                    drive.loop(0,0,0);
                    xPower = 0;
                    yPower = 0;
                    rotPower = 0;
                    currentState = STATES.ARMDOWN;
                }
                now = (drive.nowPose);
                rotPower = rotPID.calculate(now.getHeading(), trajPoseGrab.getHeading());
                xPower =  txPID.calculate(now.getX(), trajPoseGrab.getX());
                yPower = tyPID.calculate(now.getY(), trajPoseGrab.getY());
                drive.loopFC(drive.imu.getRobotYawPitchRollAngles().getYaw(), yPower, xPower, rotPower); // TODO: i switched this out of fc, it could have been wrong
                // if that doesnt work use fc with different theta
                break;
            case ARMDOWN:
                if (currentState != previousState) {
                    previousState = STATES.ARMDOWN;
                    n.getSpecimen();
                } else if (n.targetSlidePosition >= 750 & n.targPivotPos < -200 & n.isAtTargetHeight() && n.pivot.getCurrentPosition() <= -265) {
                    currentState = STATES.GRAB2;
                }
                n.loop();
                drive.loopFC(drive.imu.getRobotYawPitchRollAngles().getYaw(), 0,0,0);
                break;
            case GRAB2:
                if (currentState != previousState) {
                    previousState = STATES.GRAB2;
                    grab2Timer.reset();
                } else if (grab2Timer.seconds() > 0.5) {
                    n.grab();
                    currentState = STATES.PULLSPECIMEN;
                }
                drive.loopFC(drive.imu.getRobotYawPitchRollAngles().getYaw(), 0,0,0);
                break;
            case PULLSPECIMEN:
                if (currentState != previousState) {
                    n.groundIn();
                    walltimer.reset();
                    previousState = STATES.PULLSPECIMEN;
                } else if (walltimer.seconds() > 2) {
                    currentState = STATES.PARK;
                }
                break;
            case PARK:
                if (currentState != previousState) {
                    previousState = STATES.PARK;
                    n.goUp(0);
                    n.wristIn();
                    trajPose2 = new Pose2d(new Translation2d(-2, -36), new Rotation2d(0));
                } else if (Math.abs(now.getX() - trajPose2.getX()) < 1 & Math.abs(now.getY() - trajPose2.getY()) < 1) {
                    drive.loop(0,0,0);
                    xPower = 0;
                    yPower = 0;
                    rotPower = 0;
                    currentState = STATES.DONE;
                }
                now = (drive.nowPose);
                rotPower = rotPID.calculate(now.getHeading(), trajPose2.getHeading());
                xPower =  txPID.calculate(now.getX(), trajPose2.getX());
                yPower = tyPID.calculate(now.getY(), trajPose2.getY());
                drive.loopFC(drive.imu.getRobotYawPitchRollAngles().getYaw(), yPower, xPower, rotPower);
                break;
            case DONE:
                drive.loopFC(drive.imu.getRobotYawPitchRollAngles().getYaw(),0,0,0);
                break;
        }
        n.loop();
        n.getTelemetry(t2);
        t2.addData("State", currentState);
        t2.addData("targX", trajPose.getX());
        t2.addData("targY", trajPose.getY());
        t2.addData("targH", trajPose.getHeading());
        t2.addData("nowX", now.getX());
        t2.addData("nowY", now.getY());
        t2.addData("nowH", now.getHeading());
        t2.addData("xPower", xPower);
        t2.addData("yPower", yPower);
        t2.addData("rotPower", rotPower);
        t2.addData("trajtimer", trajTimer.seconds());
        t2.update();
    }
    @Override
    public void stop() {
        drive.stop();
    }
}
