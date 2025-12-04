package org.firstinspires.ftc.teamcode.auton;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.teamcode.odom.Chassis;
import org.firstinspires.ftc.teamcode.odom.Drivetrain;
import org.firstinspires.ftc.teamcode.odom.Odometry;

@Autonomous(name = "Small Blue", group = "Autonomous")
public class SmallBlue extends LinearOpMode {

    private Drivetrain drivetrain;
    private Odometry odometry;
    private Chassis chassis;

    @Override
    public void runOpMode() {
        DcMotor frontLeft = hardwareMap.get(DcMotor.class, "front_left_drive");
        DcMotor frontRight = hardwareMap.get(DcMotor.class, "front_right_drive");
        DcMotor backLeft = hardwareMap.get(DcMotor.class, "back_left_drive");
        DcMotor backRight = hardwareMap.get(DcMotor.class, "back_right_drive");
        DcMotor leftFlywheel = hardwareMap.get(DcMotor.class, "left_flywheel");
        DcMotor rightFlywheel = hardwareMap.get(DcMotor.class, "right_flywheel");
        leftFlywheel.setDirection(DcMotor.Direction.REVERSE);
        rightFlywheel.setDirection(DcMotor.Direction.FORWARD);

        drivetrain = new Drivetrain(frontLeft, frontRight, backLeft, backRight);
        odometry = new Odometry(hardwareMap);
        chassis = new Chassis(drivetrain, odometry);

        try {
            Thread.sleep(500);
        } catch (InterruptedException e) {
            Thread.currentThread().interrupt();
        }

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();

        chassis.initialize();
        // Let odometry settle for a moment
        try {
            Thread.sleep(500);
        } catch (InterruptedException e) {
            Thread.currentThread().interrupt();
        }

        if (!odometry.hasPinpoint()) {
            telemetry.addData("OdomComputer", "Not Found. Abort Auton.");
            telemetry.update();
            chassis.stop();
            return;
        }

        double headingRadAdj = odometry.getHeadingRadians();
        telemetry.addData("HeadingRad", "%f", headingRadAdj);
        telemetry.addData("HeadingDeg", "%f", Math.toDegrees(headingRadAdj));
        telemetry.update();

        // Put Code Here
        chassis.setPose(63, -12, 270);
        chassis.driveToPose(-24, -24, 225, 7500);
        leftFlywheel.setPower(1.0);
        rightFlywheel.setPower(1.0);
        try {
            Thread.sleep(5000);
        } catch (InterruptedException e) {
            Thread.currentThread().interrupt();
        }
        leftFlywheel.setPower(0.0);
        rightFlywheel.setPower(0.0);
        chassis.driveToPose(-36, -54, 180, 5000);
    }
}
