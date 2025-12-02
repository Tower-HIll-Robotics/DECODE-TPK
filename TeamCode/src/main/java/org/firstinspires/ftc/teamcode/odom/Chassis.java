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
    /** @noinspection FieldCanBeLocal*/
    private Thread odometryThread;

    private final Drivetrain drivetrain;
    private final Odometry odometry;
    private final PIDController lateralPID;
    private final PIDController angularPID;

    /** @noinspection unused*/
    public Chassis(Drivetrain drivetrain, Odometry odometry) {
        this.drivetrain = drivetrain;
        this.odometry = odometry;
        this.lateralPID = new PIDController(0.01, 0, 0);
        this.angularPID = new PIDController(0.01, 0, 0);
    }

    public void initialize() {
        odometry.calibrate();
        running = true;
        odometryThread = new Thread(() -> {
            while (running) {
                updatePose();
                try {
                    Thread.sleep(10);
                } catch (InterruptedException e) {
                    break;
                }
            }
        });
        odometryThread.start();
    }

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

    public void setPose(float x, float y, float theta) {
        this.x = x;
        this.y = y;
        this.theta = theta;
        odometry.setPose(x, y, theta);
    }

    public void driveToPose(float targetX, float targetY, float targetTheta, int timeout) {
        turnToHeading(angleToTarget(targetX, targetY), timeout);
        driveToPoint(targetX, targetY, timeout, true);
        turnToHeading(targetTheta, timeout);
    }

    public void driveToPoint(float targetX, float targetY, int timeout, boolean lockAngle) {
        long startTime = System.currentTimeMillis();
        lateralPID.resetIntegral();

        while (System.currentTimeMillis() - startTime < timeout) {
            float distanceX = targetX - x;
            float distanceY = targetY - y;
            float lateralError = (float)Math.hypot(distanceX, distanceY);
            if (lateralError < 0.5) {
                break;
            }

            float targetTheta = (float)Math.toDegrees(Math.atan2(distanceY, distanceX));
            float angularError = normalizeAngle(targetTheta - theta);

            float drivePower = (float)lateralPID.calculate(lateralError);
            float turnPower = (float)angularPID.calculate(angularError);

            if (lockAngle) {
                drivetrain.driveLocked(drivePower, targetTheta, theta);
            } else {
                drivetrain.drive(drivePower, 0, turnPower);
            }

            try {
                Thread.sleep(10); // avoid busy spin
            } catch (InterruptedException e) {
                Thread.currentThread().interrupt();
                break;
            }
        }

        drivetrain.stop();
    }

    public void turnToHeading(float targetTheta, int timeout) {
        long start = System.currentTimeMillis();
        angularPID.resetIntegral();

        while (System.currentTimeMillis() - start < timeout) {
            float error = normalizeAngle(targetTheta - theta);
            if (Math.abs(error) < 1.0) {
                break;
            }

            float turnPower = (float)angularPID.calculate(error);
            drivetrain.drive(0, 0, turnPower);

            try {
                Thread.sleep(10); // avoid busy spin
            } catch (InterruptedException e) {
                Thread.currentThread().interrupt();
                break;
            }
        }

        drivetrain.stop();
    }

    private void updatePose() {
        float[] pose = odometry.getPose();
        x = pose[0];
        y = pose[1];
        theta = pose[2];
    }

    private float angleToTarget(float targetX, float targetY) {
        float distanceX = targetX - x;
        float distanceY = targetY - y;
        return (float)Math.toDegrees(Math.atan2(distanceY, distanceX));
    }

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
