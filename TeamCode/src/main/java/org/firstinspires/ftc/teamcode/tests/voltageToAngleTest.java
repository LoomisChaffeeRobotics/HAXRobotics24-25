package org.firstinspires.ftc.teamcode.tests;

import androidx.annotation.RequiresPermission;

import com.acmerobotics.dashboard.FtcDashboard;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.util.ReadWriteFile;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.subsystems.swerve.voltageToAngleConstants;

import java.io.File;

@TeleOp
public class voltageToAngleTest extends OpMode {
    AnalogInput flEnc, frEnc, blEnc, brEnc;
    CRServo flAngle, frAngle, blAngle, brAngle;
    FtcDashboard dashboard;
    String[] encoderNames = {
            "fl_encoder",
            "fr_encoder",
            "bl_encoder",
            "br_encoder"
    };
    Telemetry telemetry2;
    voltageToAngleConstants converter;
    @Override
    public void init() {
        flEnc = hardwareMap.get(AnalogInput.class, "fl_encoder");
        frEnc = hardwareMap.get(AnalogInput.class, "fr_encoder");
        blEnc = hardwareMap.get(AnalogInput.class, "bl_encoder");
        brEnc = hardwareMap.get(AnalogInput.class, "br_encoder");
        converter = new voltageToAngleConstants(this, hardwareMap,encoderNames);
//
//        flAngle = hardwareMap.get(CRServo.class, "fl_angle");
//        frAngle = hardwareMap.get(CRServo.class, "fr_angle");
//        blAngle = hardwareMap.get(CRServo.class, "bl_angle");
//        brAngle = hardwareMap.get(CRServo.class, "br_angle");
        ReadWriteFile.writeFile(new File("~/FIRST/wheelAngles.txt"), "");

        dashboard = FtcDashboard.getInstance();
        telemetry2 = dashboard.getTelemetry();
    }
    @Override
    public void init_loop() {
        converter.init_loop();
    }
    @Override
    public void loop() {

        converter.loop();
        converter.getTelemetry(telemetry);
        converter.getTelemetry(telemetry2);

        telemetry.update();
        telemetry2.update();
    }
}
