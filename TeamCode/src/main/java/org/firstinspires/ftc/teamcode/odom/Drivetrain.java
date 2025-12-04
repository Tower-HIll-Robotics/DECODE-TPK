package org.firstinspires.ftc.teamcode.odom;

import com.qualcomm.robotcore.hardware.DcMotor;

/*
 * TowerPath Motion Library
 * Developed by FTC Team 22730 Tower Patch Kids
 *
 * Drivetrain.java - Class for interfacing with robot drive motors
 * Works with 4 wheel mecanum setups
 */

public class Drivetrain {
    private final DcMotor frontLeft, frontRight, backLeft, backRight;

    // Class constructor
    public Drivetrain(DcMotor frontLeft, DcMotor frontRight, DcMotor backLeft, DcMotor backRight) {
        this.frontLeft = frontLeft;
        this.frontRight = frontRight;
        this.backLeft = backLeft;
        this.backRight = backRight;

        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        frontRight.setDirection(DcMotor.Direction.REVERSE);
        backLeft.setDirection(DcMotor.Direction.FORWARD);
        backRight.setDirection(DcMotor.Direction.REVERSE);
    }

    // Drive from axial lateral yaw input
    public void drive(float forward, float strafe, float turn) {
        float fl = forward + strafe + turn;
        float fr = forward - strafe - turn;
        float bl = forward - strafe + turn;
        float br = forward + strafe - turn;

        float max = Math.max(1.0f,
                Math.max(Math.abs(fl), Math.max(Math.abs(fr), Math.max(Math.abs(bl), Math.abs(br)))));

        frontLeft.setPower(fl / max);
        frontRight.setPower(fr / max);
        backLeft.setPower(bl / max);
        backRight.setPower(br / max);
    }

    // Drive with axial targetTheta input
    public void driveLocked(float power, float targetTheta, float currentTheta) {
        double currentRad = Math.toRadians(currentTheta);
        double targetRad = Math.toRadians(targetTheta);

        float forward = (float)(power * Math.cos(targetRad - currentRad));
        float strafe = (float)(-power * Math.sin(targetRad - currentRad));

        drive(forward, strafe, 0);
    }

    public void stop() {
        drive(0, 0, 0);
    }
}
