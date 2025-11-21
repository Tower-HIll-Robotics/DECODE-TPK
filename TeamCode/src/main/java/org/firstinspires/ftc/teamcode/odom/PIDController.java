package org.firstinspires.ftc.teamcode.odom;

/*
 * TowerPath Motion Library
 * Developed by FTC Team 22730 Tower Patch Kids
 *
 * PIDController.java - Generic PID Controller class
 */

public class PIDController {
    private final double kP;
    private final double kI;
    private final double kD;
    private double integral = 0;
    private double previousError = 0;
    private long lastTime = 0;

    public PIDController(double kP, double kI, double kD) {
        this.kP = kP;
        this.kI = kI;
        this.kD = kD;
    }

    public void resetIntegral() {
        integral = 0;
        previousError = 0;
        lastTime = System.nanoTime();
    }

    public double calculate(double error) {
        long now = System.nanoTime();
        double dt = (now - lastTime) / 1e9;
        lastTime = now;

        integral += error * dt;
        double derivative = (dt > 0) ? (error - previousError) / dt : 0;
        previousError = error;

        return (kP * error) + (kI * integral) + (kD * derivative);
    }
}
