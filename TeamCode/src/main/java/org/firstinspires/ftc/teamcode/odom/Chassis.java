package org.firstinspires.ftc.teamcode.odom;

/*
 * TowerPath Motion Library
 * Developed by FTC Team 22730 Tower Patch Kids
 *
 * Chassis.java - Core class for controlling autonomous robot movement
 */

public class Chassis {
    private volatile float x;
    private volatile float y;
    private volatile float theta;

    private volatile boolean running = false;
    private Thread odometryThread;

    private final Drivetrain drivetrain;
    private final Odometry odometry;
    private final PIDController lateralPID;
    private final PIDController angularPID;

    // Class Constructor
    public Chassis(Drivetrain drivetrain, Odometry odometry) {
        this.drivetrain = drivetrain;
        this.odometry = odometry;
        this.lateralPID = new PIDController(0.01, 0, 0);
        this.angularPID = new PIDController(0.01, 0, 0);
    }

    // Initialize Chassis Devices
    public void initialize() {
        odometry.calibrate();
        try {
            Thread.sleep(300);
        } catch (InterruptedException e) {
            Thread.currentThread().interrupt();
        }

        try {
            odometry.waitForPosition(1000);
        } catch (Exception ignored) {}

        running = true;
        odometryThread = new Thread(() -> {
            while (running) {
                try {
                    updatePose();
                } catch (Exception ignored) {}
                try {
                    Thread.sleep(10);
                } catch (InterruptedException e) {
                    break;
                }
            }
        });
        odometryThread.start();
    }

    // Cancel All Motion
    public void stop() {
        running = false;
        if (odometryThread != null) {
            odometryThread.interrupt();
            try {
                odometryThread.join(200);
            } catch (InterruptedException ignored) {
                Thread.currentThread().interrupt();
            }
        }
    }

    // Set Position
    public void setPose(float x, float y, float theta) {
        this.x = x;
        this.y = y;
        this.theta = theta;
        odometry.setPose(x, y, theta);
    }

    // Drive to a specified X, Y, Theta coordinate.
    public void driveToPose(float targetX, float targetY, float targetTheta, int timeout) {
        turnToHeading(angleToTarget(targetX, targetY), timeout);
        driveToPoint(targetX, targetY, timeout, true);
        turnToHeading(targetTheta, timeout);
    }

    // Drive to a specified X, Y point. Can lock the angle (strafe)
    public void driveToPoint(float targetX, float targetY, int timeout, boolean lockAngle) {
        long startTime = System.currentTimeMillis();
        lateralPID.resetIntegral();

        while (System.currentTimeMillis() - startTime < timeout) {
            float distanceX = targetX - x;
            float distanceY = targetY - y;
            float lateralError = (float)Math.hypot(distanceX, distanceY);
            if (lateralError < 1.0) {
                break;
            }

            float targetTheta = (float)Math.toDegrees(Math.atan2(distanceY, distanceX)) - 90.0f;

            float angularError = normalizeAngle(targetTheta - theta);

            float drivePower = (float)lateralPID.calculate(lateralError);
            float turnPower = (float)angularPID.calculate(angularError);

            final float MAX_DRIVE = 0.5f;
            final float MAX_TURN = 0.35f;
            if (drivePower > MAX_DRIVE) drivePower = MAX_DRIVE;
            if (drivePower < -MAX_DRIVE) drivePower = -MAX_DRIVE;
            if (turnPower > MAX_TURN) turnPower = MAX_TURN;
            if (turnPower < -MAX_TURN) turnPower = -MAX_TURN;

            if (Math.abs(drivePower) < 0.02 && lateralError < 2.0) {
                break;
            }

            if (lockAngle) {
                drivetrain.driveLocked(drivePower, targetTheta, theta);
            } else {
                drivetrain.drive(drivePower, 0, turnPower);
            }

            try {
                Thread.sleep(10);
            } catch (InterruptedException e) {
                Thread.currentThread().interrupt();
                break;
            }
        }

        drivetrain.stop();
    }

    // Turn to a specified theta heading.
    public void turnToHeading(float targetTheta, int timeout) {
        long start = System.currentTimeMillis();
        angularPID.resetIntegral();

        while (System.currentTimeMillis() - start < timeout) {
            float error = normalizeAngle(targetTheta - theta);
            if (Math.abs(error) < 1.0) {
                break;
            }

            float turnPower = (float)angularPID.calculate(error);

            final float MAX_TURN = 0.35f;
            if (turnPower > MAX_TURN) turnPower = MAX_TURN;
            if (turnPower < -MAX_TURN) turnPower = -MAX_TURN;
            drivetrain.drive(0, 0, turnPower);

            try {
                Thread.sleep(10);
            } catch (InterruptedException e) {
                Thread.currentThread().interrupt();
                break;
            }
        }

        drivetrain.stop();
    }

    // Get current position from odom
    private void updatePose() {
        float[] pose = odometry.getPose();
        x = pose[0];
        y = pose[1];
        theta = pose[2];
    }

    // Align to drive straight to coordinate with atan2
    private float angleToTarget(float targetX, float targetY) {
        float distanceX = targetX - x;
        float distanceY = targetY - y;
        return (float)Math.toDegrees(Math.atan2(distanceY, distanceX)) - 90.0f;
    }

    // Get angle into -180 < theta < 180 range
    private float normalizeAngle(float angle) {
        while (angle > 180) {
            angle -= 360;
        }
        while (angle < -180) {
            angle += 360;
        }
        return angle;
    }
}
