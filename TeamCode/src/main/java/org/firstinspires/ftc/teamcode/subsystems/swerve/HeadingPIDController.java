package org.firstinspires.ftc.teamcode.subsystems.swerve;

public class HeadingPIDController {
    private double kP, kI, kD;
    private double integral, previousError;

    public HeadingPIDController(double kP, double kI, double kD) {
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;
        this.integral = 0;
        this.previousError = 0;
    }

    public double calculate(double targetHeading, double currentHeading) {
        double error = targetHeading - currentHeading;
        error = normalizeError(error);

        integral += error;
        double derivative = error - previousError;
        previousError = error;

        return kP * error + kI * integral + kD * derivative;
    }

    private double normalizeError(double error) {
        if (error > 180) { error -= 360; }
        if (error < -180) { error += 360; }
        return error;
    }
}