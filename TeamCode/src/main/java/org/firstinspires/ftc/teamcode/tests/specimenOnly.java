package org.firstinspires.ftc.teamcode.tests;

import com.acmerobotics.dashboard.FtcDashboard;
//import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.trajectory.Trajectory;
import com.arcrobotics.ftclib.geometry.Translation2d;
import com.arcrobotics.ftclib.trajectory.TrajectoryConfig;
import com.arcrobotics.ftclib.trajectory.TrajectoryGenerator;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.subsystems.nematocyst;
import org.firstinspires.ftc.teamcode.subsystems.swerve.SwerveDrive;
//import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
//import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequenceBuilder;
//import org.firstinspires.ftc.teamcode.trajectorysequence.sequencesegment.TrajectorySegment;

import java.util.ArrayList;
import java.util.Arrays;
import java.util.List;

@Autonomous
public class specimenOnly extends OpMode {
    SwerveDrive drive;
    nematocyst n;
    FtcDashboard dash;
    Telemetry t2;
    public static double tP = .3;
    public static double tI = 0;
    public static double tD = 0.0001;
    public static double rP = .075;
    public static double rI = 0;
    public static double rD = 0;
    double rotPower;
    double xPower;
    double yPower;
//    TrajectorySequence trajSequence;
    PIDController txPID;
    PIDController tyPID;
    PIDController rotPID;
    Pose2d now;
    Pose2d trajPose;
//    TrajectorySequenceBuilder builder;
    ElapsedTime trajTimer;
    public enum STATES {
        START,
        FORWARD,
        ARM_UP,
        PULL,
        RELEASE,
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
    public Pose2d rotateFTCLibPose(com.arcrobotics.ftclib.geometry.Pose2d odoPose) {
        Pose2d tempPose = new Pose2d(odoPose.getY()*-1,odoPose.getX(), new Rotation2d(odoPose.getHeading()+Math.PI/2));
        return tempPose;
    }
    @Override
    public void init() {
        List<Translation2d> tlist = new ArrayList<>();
        TrajectoryConfig traj1Config = new TrajectoryConfig(18, 18);
        traj1 = TrajectoryGenerator.generateTrajectory(
                new Pose2d(new Translation2d(0,0), new Rotation2d(0)),
                tlist,
                new Pose2d(new Translation2d(18,0), new Rotation2d(0)),
                traj1Config
        );

        drive = new SwerveDrive(
                11, 11, 18, 18,
                this, gamepad1, hardwareMap,
                encoderNames, driveNames, angleNames, 0, 0, 0);
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
        currentState = STATES.FORWARD;
        trajTimer.reset();
    }
    @Override
    public void loop() {
        switch (currentState) {
            case FORWARD:
                if (currentState != previousState) {
                    trajTimer.reset();
                    previousState = STATES.FORWARD;
                } else if (now.getX() > 17) {
                    currentState = STATES.ARM_UP;
                    drive.loop(0,0,0);
                }
                trajPose = traj1.sample(trajTimer.seconds()).poseMeters;
                now = (drive.nowPose);
                rotPower = -rotPID.calculate(now.getHeading(), trajPose.getHeading());
                xPower = -txPID.calculate(now.getX(), trajPose.getX());
                yPower = -tyPID.calculate(now.getY(), trajPose.getY());
                drive.loop(yPower, xPower, rotPower);
            case ARM_UP:
                if (currentState != previousState) {
                    n.goSpecimen(2);
                    n.wristSpecimen();
                    previousState = STATES.ARM_UP;
                } else if (n.slideMotor.getCurrentPosition() > 2900) {
                    currentState = STATES.PULL;
                }
                n.loop();
                break;
            case PULL:
                if (currentState != previousState) {
                    n.wristIn();
                    pullWaiter.reset();
                } else if (pullWaiter.seconds() > 5) {
                    currentState = STATES.RELEASE;
                }
            case RELEASE:
                n.release();
                currentState = STATES.PARK;
                break;
            case PARK:
//                if (currentState != previousState) {
//                    drive.loop(.5, 0, 0);
//                    timer = new ElapsedTime();
//                    timer.reset();
//                    previousState = STATES.PARK;
//                } else if ((timer.seconds() > 2)) {
                    currentState = STATES.DONE;
//                }
                break;
            case DONE:
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
        t2.update();
    }
    @Override
    public void stop() {
        drive.stop();
    }
}
