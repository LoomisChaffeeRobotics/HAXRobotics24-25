package org.firstinspires.ftc.teamcode.tests;

import com.acmerobotics.dashboard.FtcDashboard;
import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.subsystems.nematocyst;
@TeleOp
@Config

public class slideTest extends OpMode {
    nematocyst slide;
    FtcDashboard dash;
    Telemetry t2;
    public boolean isRBPressed = false;
    public boolean wasRBLastPressed;
    public static double sP = 0.0015;
    public static double sI = 0.000;
    public static double sD = 0.00;
    public static double angP = 0.0041;
    public static double angI = 0.00015;
    public static double angD = 0.0004;
    public static double angCos = 0.325;
    public static double angExt = 0.000075;
    public void buttonPressedRB() {
        if (gamepad2.right_bumper && !isRBPressed) {
            isRBPressed = true;
            wasRBLastPressed = false;
        }
        if (!gamepad2.right_bumper && isRBPressed) {
            isRBPressed = false;
            wasRBLastPressed = true;
        }
    }
    @Override
    public void init() {
        slide = new nematocyst(this);
        slide.init("pivot", "slide", "wrist", "claw");
        dash = FtcDashboard.getInstance();
        t2 = dash.getTelemetry();
    }

    @Override
    public void loop() {
        if (gamepad2.a || gamepad2.cross) {
            slide.goUp(0);
        } else if (gamepad2.x || gamepad2.square) {
            slide.goGround(13.0);
        } else if (gamepad2.b || gamepad2.circle) {
            slide.goSpecimen(28);
        } else if (gamepad2.y || gamepad2.triangle) {
            slide.goOut(36.0);
        } else if (gamepad2.right_trigger > 0.5) {
            slide.goSpecimenDown(24);
        } else if (gamepad2.left_trigger > 0.5) {
            slide.groundIn();
        }

        buttonPressedRB();
        if (wasRBLastPressed) {
            slide.switchClaw();
            wasRBLastPressed = false;
        }

        if (gamepad2.dpad_down) {
            slide.wristIn();
        } else if (gamepad2.dpad_up) {
            slide.wristOut();
        } else if (gamepad2.dpad_left) {
            slide.wristDown();
        } else if (gamepad2.dpad_right) {
            slide.wristSpecimen();
        }

        slide.loop(sP, sI, sD);
        slide.updatePID(angP, angI, angD, angCos, angExt);
        slide.updateSlidePID(sP, sI, sD);
        slide.getTelemetry();
        slide.getTelemetry(t2);

    }
}
