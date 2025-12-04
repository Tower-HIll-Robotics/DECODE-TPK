package org.firstinspires.ftc.teamcode.odom;

import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.HardwareMap;

/*
 * TowerPath Motion Library
 * Developed by FTC Team 22730 Tower Patch Kids
 *
 * Odometry.java - Class for interfacing with GoBilda Odometry system
 * Works with GoBilda Pinpoint Odometry Computer with 2 tracking wheels
 */

public class Odometry {
    private GoBildaPinpointDriver pinpoint = null;

    private volatile double x = 0;
    private volatile double y = 0;
    private volatile double theta = 0;
    private volatile Pose2D position = null;

    // Class Constructor
    public Odometry(HardwareMap hwMap) {
        try {
            pinpoint = hwMap.get(GoBildaPinpointDriver.class, "pinpoint");
        } catch (Exception ignored) {
            pinpoint = null;
        }
    }

    // Recalibrate Sensors
    public void calibrate() {
        if (pinpoint != null) {
            try {
                pinpoint.resetPosAndIMU();
            } catch (Exception ignored) {}
        }
    }

    // Wait to see if Odom gets a reading
    public boolean waitForPosition(long timeoutMs) {
        if (pinpoint == null) return false;
        long start = System.currentTimeMillis();
        while (System.currentTimeMillis() - start < timeoutMs) {
            try {
                pinpoint.update();
                Pose2D pos = pinpoint.getPosition();
                if (pos != null) {
                    double rawX = pos.getX(DistanceUnit.INCH);
                    double rawY = pos.getY(DistanceUnit.INCH);
                    this.x = rawX;
                    this.y = rawY;
                    this.theta = pos.getHeading(AngleUnit.DEGREES) - 90.0;
                    this.position = pos;
                    return true;
                }
            } catch (Exception ignored) {}
            try {
                Thread.sleep(20);
            } catch (InterruptedException e) {
                Thread.currentThread().interrupt();
                break;
            }
        }
        return false;
    }

    // Return current position
    public float[] getPose() {
        if (pinpoint != null) {
            try {
                pinpoint.update();
                Pose2D pos = pinpoint.getPosition();

                if (pos != null) {
                    double xInches = pos.getX(DistanceUnit.INCH);
                    double yInches = pos.getY(DistanceUnit.INCH);
                    double headingDeg = pos.getHeading(AngleUnit.DEGREES) - 90.0;

                    this.x = xInches;
                    this.y = yInches;
                    this.theta = headingDeg;
                    this.position = pos;

                    return new float[]{(float) xInches, (float) yInches, (float) headingDeg};
                }
            } catch (Exception ignored) {}
        }

        return new float[]{(float) x, (float) y, (float) theta};
    }

    // Sets the Pose DO NOT CALL THIS FROM OUTSIDE OF THE CHASSIS CLASS
    public void setPose(double x, double y, double theta) {
        this.x = x;
        this.y = y;
        this.theta = theta;
        if (pinpoint != null) {
            try {
                double deviceTheta = theta + 90.0;
                Pose2D pose = new Pose2D(DistanceUnit.INCH, x, y, AngleUnit.DEGREES, deviceTheta);
                pinpoint.setPosition(pose);
                this.position = pose;
            } catch (Exception ignored) {}
        } else {
            this.position = new Pose2D(DistanceUnit.INCH, x, y, AngleUnit.DEGREES, theta);
        }
    }

    // Check if odom computer is present
    public boolean hasPinpoint() {
        return pinpoint != null;
    }

    // Return current heading in Radians (Java Math uses Radians 🤮)
    public double getHeadingRadians() {
        if (pinpoint != null) {
            try {
                return pinpoint.getHeading(AngleUnit.RADIANS) - Math.toRadians(90);
            } catch (Exception ignored) {}
        }
        return Math.toRadians(this.theta);
    }
}
