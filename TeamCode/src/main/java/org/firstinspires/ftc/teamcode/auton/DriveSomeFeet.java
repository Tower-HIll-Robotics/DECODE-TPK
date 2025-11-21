package org.firstinspires.ftc.teamcode.auton;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import org.firstinspires.ftc.teamcode.odom.Chassis;
import org.firstinspires.ftc.teamcode.odom.Odometry;
import org.firstinspires.ftc.teamcode.odom.Drivetrain;

@Autonomous(name = "Drive Some Feet", group = "Autonomous")
public class DriveSomeFeet extends LinearOpMode {

    private Drivetrain drivetrain;
    private Odometry odometry;
    private Chassis chassis;

    @Override
    public void runOpMode() {
        DcMotor frontLeft = hardwareMap.get(DcMotor.class, "front_left_drive");
        DcMotor frontRight = hardwareMap.get(DcMotor.class, "front_right_drive");
        DcMotor backLeft = hardwareMap.get(DcMotor.class, "back_left_drive");
        DcMotor backRight = hardwareMap.get(DcMotor.class, "back_right_drive");

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

        chassis.setPose(0, 0, 0);

        chassis.driveToPose(0, 48, 0, 5000);

        drivetrain.stop();
        chassis.stop();
    }
}
