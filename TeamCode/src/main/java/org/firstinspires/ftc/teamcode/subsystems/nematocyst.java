package org.firstinspires.ftc.teamcode.subsystems;

import com.acmerobotics.dashboard.config.Config;
import com.arcrobotics.ftclib.controller.PIDController;
import com.arcrobotics.ftclib.controller.PIDFController;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@Config
public class nematocyst {
    public DcMotor pivot;
    public DcMotor slideMotor;
    public Servo claw;
    public Servo wrist;
    double pivotPower;
//    private static final int degreesToTicks = 1425 /360;
//    private static final double inchesToDegrees =  1/2.067;
    private static final double ticksPerInch = (double) 1425/(Math.PI * 3.75); // TODO: wtf???
    private static final double Max_Extension = 22 * ticksPerInch; // Horizontal Max Ticks
    private static final double degPerTick = (double) 360/1425;

    // PID constants
    public double sP = 0.0015;
    public double sI = 0.0001;
    public double sD = 0.00;
    public double pP = 0.0044;
    public double pI = 0.0002;
    public double pD = 0.0004;
    public double pCos = 0.325;
    public double pExt = 0.00006;
    boolean isTargAngDown = false;
    boolean justWasTargAngDown = false;
    PIDFController slidePID;
//    ElapsedTime pidTimer;
    // PID variables
    private double integral = 0;
    private double lastError = 0;
    private double targSlideHeight = 0;
    private int targetSlidePosition = (int) (targSlideHeight * ticksPerInch);  // Starting at initial position (adjust)
    private int targPivotPos;
//    private int maxSlidePos;
    private int maxPivotPos = -350;
    boolean isPullingDown = false;
    boolean lastWasClosed = true;
    ElapsedTime pivotTimer;
    public boolean activelyPullingDown = false;
    OpMode opMode;
    public nematocyst(OpMode OM) {
        opMode = OM;
    }
    public void init(String pivotName, String slideName, String wristName, String clawName) {
        slideMotor = opMode.hardwareMap.get(DcMotor.class, slideName);
        pivot = opMode.hardwareMap.get(DcMotor.class, pivotName);
        claw = opMode.hardwareMap.get(Servo.class, clawName);
        wrist = opMode.hardwareMap.get(Servo.class, wristName);
        slideMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER); // sets the STARTING pos to 0
        pivot.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        slideMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        pivot.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER); // don't "try its best to run at a velocity" do it ourselves
        slideMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        pivot.setDirection(DcMotorSimple.Direction.REVERSE);
        pivot.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        slideMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
//        maxSlidePos = (int) Math.round(Max_Extension);
        targSlideHeight = 0;
        targPivotPos = 0;
        pivotTimer = new ElapsedTime();
        slidePID = new PIDController(sP, sI, sD);
        grab();
        wristIn();
    }

    public void loop(double P, double I, double D) {
        targPivotPos = Math.min(0, Math.max(targPivotPos, maxPivotPos));
        // Ensure the motor does not exceed max forward extension
        double out;
        if (!isPullingDown) {
            if (!justWasTargAngDown) {
                out = slidePID.calculate(slideMotor.getCurrentPosition(), targetSlidePosition);
                pivotPower = calculatePID(targPivotPos, pivot.getCurrentPosition());
            } else {
                out = slidePID.calculate(slideMotor.getCurrentPosition(), 0);
                if (slideMotor.getCurrentPosition() > 300) {
                    pivotPower = 0;
                } else {
                    justWasTargAngDown = false;
                }
            }
        } else {
            out = slidePID.calculate(slideMotor.getCurrentPosition(), targetSlidePosition);
            pivotPower = calculatePID(targPivotPos, pivot.getCurrentPosition());
        }
        if (activelyPullingDown) {
            out = -1;
        }
//        SlewRateLimiter slideLimiter = new SlewRateLimiter(0.5); //TODO: Tune this
        slidePID.setPIDF(P, I, D,0);
        slideMotor.setPower(out);
        pivot.setPower(pivotPower); // use updatdePID to fix this
    }
    public void loop() {
        this.loop(sP, sI, sD);
    }
//    public double getHypotenuseMax(double height) {
//        double in_inch;
//        in_inch=Math.sqrt(height*height+22*22);
//        return in_inch*ticksPerInch;
//    }
//    public double getHypotenuseSpecimen(double height) {
//        double in_inch;
//        in_inch=Math.sqrt(height*height+10*10);
//        return in_inch*ticksPerInch;
//    }
    public void hangReal() {
        targPivotPos = -15;
        isPullingDown = false;
        activelyPullingDown = true;
    }
    public void updatePID(double aP, double aI, double aD, double aCos, double aExt) {
        pP = aP;
        pI = aI;
        pD = aD;
        pCos = aCos;
        pExt = aExt;
    }
    public void updateSlidePID(double P, double I, double D) {
        slidePID.setPIDF(P, I, D, 0);
    }
    public void goUp(double inches) {
        if (isTargAngDown) {
            justWasTargAngDown = true;
        }
        isTargAngDown = false;
        targPivotPos = -15;
        targSlideHeight = inches;
        targetSlidePosition = (int) (targSlideHeight * ticksPerInch);
        isPullingDown = false;
    }
    public void goOut(double inches) {
//        isTargAngDown = false;
//        targSlideHeight = (1/ticksPerInch) * getHypotenuseMax(inches);
//        double pivAng = 15 + 90 - Math.asin(inches/targSlideHeight);
//        targPivotPos = (int) (-5 - (pivAng * 105/190));
//        targetSlidePosition = (int) (targSlideHeight * ticksPerInch);
//
        targetSlidePosition = 6200;
        targPivotPos = -80;
        if (isTargAngDown) {
            justWasTargAngDown = true;
        }
        isTargAngDown = false;
        isPullingDown = false;
    }
    public void touchBar() {
        targPivotPos = -50;
        targetSlidePosition = 1500;
        isPullingDown = true;
        isTargAngDown = false;
    }
    public void goSpecimen(double inches) {
//        isTargAngDown = false;
//        targSlideHeight = (1/ticksPerInch) * getHypotenuseSpecimen(inches);
//        double pivAng = 15 + 90 - Math.asin(inches/targSlideHeight);
//        targPivotPos = (int) (-5 - (pivAng * 105/190));
//        targetSlidePosition = (int) (targSlideHeight * ticksPerInch);
        targPivotPos = -80;
        targetSlidePosition = 3200;
        if (isTargAngDown) {
            justWasTargAngDown = true;
        }
        isTargAngDown = false;
        isPullingDown = false;
    }
    public void goGround(double inches) {

        targPivotPos = -355;
        targetSlidePosition = 2750;
        isTargAngDown = true;
        if (isTargAngDown) {
            justWasTargAngDown = false;
        }
        wristOut();
        isPullingDown = false;
    }
    public void getSpecimen() {
        targPivotPos = -300;
        targetSlidePosition = 500;
        if (isTargAngDown) {
            justWasTargAngDown = false;
        }
        isTargAngDown = false;
        wristOut();
        isPullingDown = false;
    }
//    public void groundIn() {
//        targPivotPos = -355;
//        targetSlidePosition = 0;
//        isTargAngDown = true;
//        if (isTargAngDown) {
//            justWasTargAngDown = false;
//        }
//        isPullingDown = false;
//    }
//    public void groundOutTest() {
//        targPivotPos = -355;
//        targetSlidePosition = 3500;
//        isTargAngDown = true;
//        if (isTargAngDown) {
//            justWasTargAngDown = false;
//        }
//        wristOut();
//    }
    public boolean isAtTargetHeight() {
        if (Math.abs(slideMotor.getCurrentPosition()/ticksPerInch - targSlideHeight) < 2) {
            return true;
        } else {
            return false;
        }
    }
    public boolean isAtTargAng() {
        if (Math.abs(pivot.getCurrentPosition() - targPivotPos) < 10) {
            return true;
        } else {
            return false;
        }
    }
    // Initialize motor
    public void manualOut() {
        targetSlidePosition += 10;
    }
    public void manualIn() {
        targetSlidePosition -= 10;
    }
    public void manualUp() { targPivotPos++;}
    public void manualDown() { targPivotPos--;}
    public void switchClaw() {
        if (lastWasClosed) {
            release();
        } else {
            grab();
        }
    }
    public void grab() { claw.setPosition(0.25); lastWasClosed = true; }
    public void release() {claw.setPosition(0.7); lastWasClosed = false;}
    public void wristOut() {wrist.setPosition(.45);}
    public void wristIn() {wrist.setPosition(1);}
    public void wristDown() {wrist.setPosition(.825);}
    public void wristSpecimen() { wrist.setPosition(.65); }



            // Clamp target position to a safe range (adjust limits as needed)
    public void getTelemetry() {
        // Telemetry for debugging
        opMode.telemetry.addData("Target Slide Position", targetSlidePosition);
        opMode.telemetry.addData("Current Slide Position", slideMotor.getCurrentPosition());
        opMode.telemetry.addData("Targ Slide Height", targSlideHeight);
        opMode.telemetry.addData("CUrrent SLIde Height", slideMotor.getCurrentPosition()/(ticksPerInch));
        opMode.telemetry.addData("Slide Power", slideMotor.getPower());
        opMode.telemetry.addData("Wrist pos", wrist.getPosition());
        opMode.telemetry.addData("Claw pos", claw.getPosition());
        opMode.telemetry.addData("Pivot Targ", targPivotPos);
        opMode.telemetry.addData("Pivot Pos", pivot.getCurrentPosition());
        opMode.telemetry.addData("Pivot Power", pivotPower);
        opMode.telemetry.addData("Claw", claw.getPosition());
        opMode.telemetry.update();
    }
    public void getTelemetry(Telemetry t) {
        // Telemetry for debugging
        t.addData("Target Slide Position", targetSlidePosition);
        t.addData("Current Slide Position", slideMotor.getCurrentPosition());
        t.addData("Targ Slide Height", targSlideHeight);
        t.addData("CUrrent SLIde Height", slideMotor.getCurrentPosition()/(ticksPerInch));
        t.addData("Slide Power", slideMotor.getPower());
        t.addData("Wrist pos", wrist.getPosition());
        t.addData("Claw pos", claw.getPosition());
        t.addData("Pivot Targ", targPivotPos);
        t.addData("Pivot Pos", pivot.getCurrentPosition());
        t.addData("Pivot Power", pivotPower);
        t.update();
    }
    private double calculatePID(int target, int current) {
        // Calculate error

        double error = target - current;
        integral += (error * pivotTimer.seconds());
//        double iTerm = i * integral;
        // Derivative term
        double derivative = ((error - lastError)/pivotTimer.seconds());
        lastError = error;
        // PID output
        double ff;
        double output;
        if (isTargAngDown) {
            if (pivot.getCurrentPosition() >= -280) {
                ff = ((pExt * slideMotor.getCurrentPosition()) - pCos * Math.cos(Math.toRadians(85 - (current * degPerTick))));
                output = (error * 0.0017) + ff;
            } else {
             output = 0;
            }
        } else {
            ff = ((pExt * slideMotor.getCurrentPosition()) - pCos * Math.cos(Math.toRadians(85 - (current * degPerTick))));
            output = (error * pP) + (integral * pI) + (derivative * pD) + (ff);
        }

        pivotTimer.reset();
        // Limit the output to motor range [-1, 1]
        output = Math.max(-1, Math.min(output, 1));
        return output;
    }
}
