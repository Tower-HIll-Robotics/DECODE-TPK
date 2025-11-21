package org.firstinspires.ftc.teamcode.odom;

import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.HardwareMap;

/*
 * TowerPath Motion Library
 * Developed by FTC Team 22730 Tower Patch Kids
 *
 * Odometry.java - Class for interfacing with GoBilda Odometry system
 * Works with GoBilda Pinpoint Odometry Computer with 2 tracking wheels
 */

public class Odometry {
    private final GoBildaPinpointDriver pinpoint;

    private volatile double x = 0;
    private volatile double y = 0;
    private volatile double theta = 0;
    private volatile Pose2D position = null;

    public Odometry(HardwareMap hwMap) {
        pinpoint = hwMap.get(GoBildaPinpointDriver.class, "pinpoint");
    }

    public void calibrate() {
        pinpoint.resetPosAndIMU();
    }

    public float[] getPose() {
        Pose2D pos = pinpoint.getPosition();

        double xInches = pos.getX(DistanceUnit.INCH);
        double yInches = pos.getY(DistanceUnit.INCH);
        double headingDeg = pos.getHeading(AngleUnit.DEGREES);

        return new float[] {(float) xInches, (float) yInches, (float) headingDeg};
    }

    public void setPose(double x, double y, double theta) {
        Pose2D pose = new Pose2D(DistanceUnit.INCH, x, y, AngleUnit.DEGREES, theta);
        pinpoint.setPosition(pose);
    }
}
