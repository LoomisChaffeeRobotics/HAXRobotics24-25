package org.firstinspires.ftc.teamcode.tests;

import com.acmerobotics.dashboard.FtcDashboard;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.geometry.Rotation2d;
import com.arcrobotics.ftclib.geometry.Translation2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.subsystems.nematocyst;
import org.firstinspires.ftc.teamcode.subsystems.swerve.SwerveDrive;

@Autonomous
public class specimenWithDelay extends OpMode {
    SwerveDrive drive;
    nematocyst n;
    FtcDashboard dash;
    Telemetry t2;
    public static double tP = .15;
    public static double tI = 0.001;
    public static double tD = 0.0001;
    public static double rP = .0375;
    public static double rI = 0;
    public static double rD = 0.0001;
    double rotPower = 0;
    double xPower = 0;
    double yPower = 0;
    PIDController txPID;
    PIDController tyPID;
    PIDController rotPID;
    Pose2d now = new Pose2d(new Translation2d(0,0), new Rotation2d(0));
    Pose2d trajPose = new Pose2d(new Translation2d(0,0), new Rotation2d(0));
    ElapsedTime trajTimer;
    ElapsedTime tempBrokenArmTimer;
    Pose2d trajPose2;
     Pose2d trajPoseB;

    public enum STATES {
        START,
        FORWARD,
        ARM_UP,
        PULL,
        BACK,
        RELEASE,
        PARK,
        DONE
    }
    STATES currentState = STATES.START;
    STATES previousState = STATES.START;
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
    ElapsedTime pauseBegin;
    @Override
    public void init() {

        drive = new SwerveDrive(
                11, 11, 18, 18,
                this, gamepad1, hardwareMap,
                encoderNames, driveNames, angleNames, 0, 0, Math.PI);

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
    }
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
                if (pauseBegin.seconds() > 10) {
                    currentState = STATES.FORWARD;
                }
                break;
            case FORWARD:
                if (currentState != previousState) {
                    trajPose = new Pose2d(new Translation2d(17, 0), new Rotation2d(0));
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
                    n.goSpecimen(2);
                    n.wristOut();
                    previousState = STATES.ARM_UP;
                } else if (n.targetSlidePosition > 2700 &
                        n.slideMotor.getCurrentPosition() > 2700
//                        tempBrokenArmTimer.seconds() > 2
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
                    currentState = STATES.BACK;
                }
                drive.loop(0,0,0);
                break;
            case BACK:
                if (currentState != previousState) {
                    trajPoseB = new Pose2d(new Translation2d(15, 0), new Rotation2d(0));
                    previousState = STATES.BACK;
                } else if (Math.abs(now.getX() - trajPoseB.getX()) < 1) {
                    drive.loop(0,0,0);
                    xPower = 0;
                    yPower = 0;
                    rotPower = 0;
                    currentState = STATES.RELEASE;
                }
                now = (drive.nowPose);
                rotPower = 0;
                xPower =  txPID.calculate(now.getX(), trajPoseB.getX());
                yPower = tyPID.calculate(now.getY(), trajPoseB.getY());
                drive.loop(yPower, xPower, rotPower);
                 break;
            case RELEASE:
                n.release();
                n.goUp(0);
                drive.loop(0,0,0);
                if (pauseBegin.seconds() > 23) {
                    currentState = STATES.PARK;
                }
                break;
            case PARK:
                if (currentState != previousState) {
                    previousState = STATES.PARK;
                    trajPose2 = new Pose2d(new Translation2d(4, -36), new Rotation2d(0));
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
