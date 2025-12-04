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

    public Odometry(HardwareMap hwMap) {
        try {
            pinpoint = hwMap.get(GoBildaPinpointDriver.class, "pinpoint");
        } catch (Exception ignored) {
            pinpoint = null;
        }
    }

    public void calibrate() {
        if (pinpoint != null) {
            try {
                pinpoint.resetPosAndIMU();
            } catch (Exception ignored) {}
        }
    }

    public float[] getPose() {
        if (pinpoint != null) {
            try {
                pinpoint.update();
                Pose2D pos = pinpoint.getPosition();

                if (pos != null) {
                    double xInches = pos.getX(DistanceUnit.INCH);
                    double yInches = pos.getY(DistanceUnit.INCH);
                    double headingDeg = pos.getHeading(AngleUnit.DEGREES);

                    this.x = xInches;
                    this.y = yInches;
                    this.theta = headingDeg;
                    this.position = pos;

                    return new float[] {(float) xInches, (float) yInches, (float) headingDeg};
                }
            } catch (Exception ignored) {}
        }
        return new float[] {(float) x, (float) y, (float) theta};
    }

    public void setPose(double x, double y, double theta) {
        this.x = x;
        this.y = y;
        this.theta = theta;
        this.position = new Pose2D(DistanceUnit.INCH, x, y, AngleUnit.DEGREES, theta);

        if (pinpoint != null) {
            try {
                Pose2D pose = new Pose2D(DistanceUnit.INCH, x, y, AngleUnit.DEGREES, theta);
                pinpoint.setPosition(pose);
            } catch (Exception ignored) {}
        }
    }

    public boolean hasPinpoint() {
        return pinpoint != null;
    }

    public double getHeadingRadians() {
        if (pinpoint != null) {
            try {
                return pinpoint.getHeading(AngleUnit.RADIANS) - Math.toRadians(90);
            } catch (Exception ignored) {}
        }
        return Math.toRadians(this.theta) - Math.toRadians(90);
    }
}
