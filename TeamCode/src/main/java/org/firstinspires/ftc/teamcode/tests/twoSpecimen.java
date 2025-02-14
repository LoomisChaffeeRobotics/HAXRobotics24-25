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
import org.firstinspires.ftc.teamcode.R;
import org.firstinspires.ftc.teamcode.subsystems.nematocyst;
import org.firstinspires.ftc.teamcode.subsystems.swerve.SwerveDrive;

import java.util.ArrayList;
import java.util.List;

@Autonomous
public class twoSpecimen extends OpMode {
    SwerveDrive drive;
    nematocyst n;
    FtcDashboard dash;
    Telemetry t2;
    public static double tP = .11;
    public static double tI = 0.0001;
    public static double tD = 0.0001;
    public static double rP = .0375;
    public static double rI = 0;
    public static double rD = 0.0001;
    double rotPower = 0;
    double xPower = 0;
    double yPower = 0;
//    TrajectorySequence trajSequence;
    PIDController txPID;
    PIDController tyPID;
    PIDController rotPID;
    Pose2d now = new Pose2d(new Translation2d(0,0), new Rotation2d(0));
    Pose2d trajPose = new Pose2d(new Translation2d(0,0), new Rotation2d(0));
//    TrajectorySequenceBuilder builder;
    ElapsedTime trajTimer;
    ElapsedTime tempBrokenArmTimer;
    Pose2d trajPose2;
    private Pose2d trajPoseGrab;
    ElapsedTime grab2Timer;
    private Pose2d trajPoseGoBack;

    public enum STATES {
        START,
        FORWARD,
        ARM_UP,
        PULL,
        RELEASE,
        DRIVEWALL,
        ARMDOWN,
        GRAB2,
        GOBACK,
        ARMUP2,
        PULL2,
        RELEASE2,
        PARK,
        DONE
    }
    STATES currentState = STATES.START;
    STATES previousState = STATES.START;
    ElapsedTime timer;
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
        rotPID = new PIDController(rP, rI, rD);
        trajTimer = new ElapsedTime();
        now = drive.nowPose;
        pullWaiter = new ElapsedTime();
        tempBrokenArmTimer = new ElapsedTime();
        grab2Timer = new ElapsedTime();
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
                    trajPose = new Pose2d(new Translation2d(12, 0), new Rotation2d(0));
                    trajTimer.reset();
                    previousState = STATES.FORWARD;
                } else if (Math.abs(now.getX() - trajPose.getX()) < 1) {
                    currentState = STATES.ARM_UP;
                    drive.loop(0,0,0);
                    xPower = 0;
                    yPower = 0;
                    rotPower = 0;
                }
                now = (drive.nowPose);
                rotPower = rotPID.calculate(now.getHeading(), trajPose.getHeading());
                xPower =  txPID.calculate(now.getX(), trajPose.getX());
                yPower = tyPID.calculate(now.getY(), trajPose.getY());
                drive.loop(yPower, xPower, rotPower);
                break;
            case ARM_UP:
                if (currentState != previousState) {
//                    n.goSpecimen(2);
                    tempBrokenArmTimer.reset();
                    n.wristOut();
                    previousState = STATES.ARM_UP;
                } else if (
//                        n.slideMotor.getCurrentPosition() > 2900
                        tempBrokenArmTimer.seconds() > 2
                ) {
                    currentState = STATES.PULL;
                }
                n.loop();
                drive.loop(0,0,0);
                break;
            case PULL:
                if (currentState != previousState) {
                    n.wristIn();
                    pullWaiter.reset();
                    previousState = STATES.PULL;
                } else if (pullWaiter.seconds() > 2) {
                    currentState = STATES.RELEASE;
                }
                drive.loop(0,0,0);
                break;
            case RELEASE:
                drive.loop(0,0,0);
                n.release();
                currentState = STATES.ARMDOWN;
                break;
            case DRIVEWALL:
                if (currentState != previousState) {
                    trajPoseGrab = new Pose2d(new Translation2d(6, -26), new Rotation2d(Math.PI));
                    previousState = STATES.DRIVEWALL;
                } else if (Math.abs(trajPoseGrab.getX() - now.getX()) < 1 & Math.abs(trajPoseGrab.getY() - now.getY()) < 1 & Math.abs(trajPoseGrab.getHeading() - now.getHeading()) < .4) {
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
                drive.loop(yPower, xPower, rotPower);
                break;
            case ARMDOWN:
                if (currentState != previousState) {
                    previousState = STATES.ARMDOWN;
                    n.getSpecimen();
                } else if (n.isAtTargetHeight() && n.isAtTargAng()) {
                    currentState = STATES.GRAB2;
                }
                n.loop();
                drive.loop(0,0,0);
                break;
            case GRAB2:
                if (currentState != previousState) {
                    previousState = STATES.GRAB2;
                    grab2Timer.reset();
                    n.grab();
                } else if (grab2Timer.seconds() > 0.5) {
                    currentState = STATES.GOBACK;
                }
                drive.loop(0,0,0);
                break;
            case GOBACK:
                if (currentState != previousState) {
                    previousState = STATES.GOBACK;
                    n.goUp(0);
                    trajPoseGoBack = new Pose2d(new Translation2d(12, 0), new Rotation2d(0));
                } else if (Math.abs(now.getX() - trajPoseGoBack.getX()) < 1 & Math.abs(now.getY() - trajPoseGoBack.getY()) < 1 & Math.abs(now.getHeading() - trajPoseGoBack.getHeading()) < .4) {
                    drive.loop(0,0,0);
                    xPower = 0;
                    yPower = 0;
                    rotPower = 0;
                    currentState = STATES.ARMUP2;
                }
                now = (drive.nowPose);
                rotPower = rotPID.calculate(now.getHeading(), trajPoseGoBack.getHeading());
                xPower =  txPID.calculate(now.getX(), trajPoseGoBack.getX());
                yPower = tyPID.calculate(now.getY(), trajPoseGoBack.getY());
                drive.loop(yPower, xPower, rotPower);
                break;
            case ARMUP2:
                if (currentState != previousState) {
                    previousState = STATES.ARMUP2;
                    n.goSpecimen(23);
                    n.wristOut();
                } else if (n.isAtTargAng() & n.isAtTargetHeight()) {
                    currentState = STATES.PULL2;
                }
                drive.loop(0,0,0);
                break;
            case PULL2:
                if (currentState != previousState) {
                    n.wristIn();
                    pullWaiter.reset();
                    previousState = STATES.PULL2;
                } else if (pullWaiter.seconds() > 2) {
                    currentState = STATES.RELEASE2;
                }
                drive.loop(0,0,0);
                break;
            case RELEASE2:
                n.release();
                drive.loop(0,0,0);
                currentState = STATES.PARK;
                break;
            case PARK:
                if (currentState != previousState) {
                    previousState = STATES.PARK;
                    n.goUp(0);
                    trajPose2 = new Pose2d(new Translation2d(0, -26), new Rotation2d(0));
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
                drive.loop(yPower, xPower, rotPower);
                break;
            case DONE:
                drive.loop(0,0,0);
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
        t2.addData("trajtimer", trajTimer.seconds());
        t2.update();
    }
    @Override
    public void stop() {
        drive.stop();
    }
}
