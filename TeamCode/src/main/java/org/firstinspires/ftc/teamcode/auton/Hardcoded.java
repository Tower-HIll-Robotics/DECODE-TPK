package org.firstinspires.ftc.teamcode.auton;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

@Autonomous(name = "Hardcoded", group = "Autonomous")
public class Hardcoded extends LinearOpMode {
    private DcMotor frontLeft = null;
    private DcMotor backLeft = null;
    private DcMotor frontRight = null;
    private DcMotor backRight = null;
    private DcMotor leftFlywheel = null;
    private DcMotor rightFlywheel = null;
    private DcMotor intake = null;
    private DcMotor intakeIntermediate = null;

    // Constants
    private static final double WHEEL_DIAMETER_INCHES = 2.0; // per user request
    // Assumption: encoder ticks per motor revolution. If you know your motor's exact value, replace this.
    private static final double COUNTS_PER_REV = 1120.0; // assumption (common value); adjust if needed
    private static final double GEAR_REDUCTION = 1.0;
    private static final double COUNTS_PER_INCH = (COUNTS_PER_REV * GEAR_REDUCTION) / (Math.PI * WHEEL_DIAMETER_INCHES);

    @Override
    public void runOpMode() {
        // assign to class fields (do not shadow)
        frontLeft = hardwareMap.get(DcMotor.class, "front_left_drive");
        frontRight = hardwareMap.get(DcMotor.class, "front_right_drive");
        backLeft = hardwareMap.get(DcMotor.class, "back_left_drive");
        backRight = hardwareMap.get(DcMotor.class, "back_right_drive");
        leftFlywheel = hardwareMap.get(DcMotor.class, "left_flywheel");
        rightFlywheel = hardwareMap.get(DcMotor.class, "right_flywheel");
        intake = hardwareMap.get(DcMotor.class, "intake");
        intakeIntermediate = hardwareMap.get(DcMotor.class, "intakeIntermediate");

        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        backLeft.setDirection(DcMotor.Direction.FORWARD);
        frontRight.setDirection(DcMotor.Direction.REVERSE);
        backRight.setDirection(DcMotor.Direction.REVERSE);
        leftFlywheel.setDirection(DcMotor.Direction.REVERSE);
        rightFlywheel.setDirection(DcMotor.Direction.FORWARD);
        intake.setDirection(DcMotor.Direction.FORWARD);
        intakeIntermediate.setDirection(DcMotor.Direction.REVERSE);

        frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        waitForStart();

        // Drive reverse for 4 feet (48 inches). Negative for reverse.
        driveInches(-36.0, 0.5);

        // Strafe left a couple inches (3 inches)
        //strafeLeftInches(3.0, 0.4);

        // Run flywheels and intakes forward for 10 seconds
        leftFlywheel.setPower(1.0);
        rightFlywheel.setPower(1.0);
        intake.setPower(1.0);
        intakeIntermediate.setPower(1.0);

        telemetry.addData("Action", "Running flywheels and intakes for 10s");
        telemetry.update();

        ElapsedTime timer = new ElapsedTime();
        while (opModeIsActive() && timer.seconds() < 10.0) {
            // keep running
            sleep(50);
        }

        // stop flywheels and intakes
        leftFlywheel.setPower(0);
        rightFlywheel.setPower(0);
        intake.setPower(0);
        intakeIntermediate.setPower(0);

        telemetry.addData("Status", "Done");
        telemetry.update();
    }

    // Drive straight using encoders. Positive inches = forward, negative = backward.
    private void driveInches(double inches, double power) {
        int moveCounts = (int) Math.round(inches * COUNTS_PER_INCH);

        // Reset and set target
        frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        frontLeft.setTargetPosition(moveCounts);
        frontRight.setTargetPosition(moveCounts);
        backLeft.setTargetPosition(moveCounts);
        backRight.setTargetPosition(moveCounts);

        frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        double absPower = Math.abs(power);
        frontLeft.setPower(absPower);
        frontRight.setPower(absPower);
        backLeft.setPower(absPower);
        backRight.setPower(absPower);

        // Timeout to avoid getting stuck: 5 seconds minimum + 1s per foot
        double timeoutS = Math.max(5.0, Math.abs(inches) / 12.0 * 1.0 + 5.0);
        ElapsedTime runtime = new ElapsedTime();

        while (opModeIsActive() && runtime.seconds() < timeoutS &&
                (frontLeft.isBusy() || frontRight.isBusy() || backLeft.isBusy() || backRight.isBusy())) {
            telemetry.addData("Driving", "target=%d, fl=%d, fr=%d, bl=%d, br=%d", moveCounts,
                    frontLeft.getCurrentPosition(), frontRight.getCurrentPosition(), backLeft.getCurrentPosition(), backRight.getCurrentPosition());
            telemetry.update();
            sleep(20);
        }

        // stop
        frontLeft.setPower(0);
        frontRight.setPower(0);
        backLeft.setPower(0);
        backRight.setPower(0);

        frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    // Strafe left a given number of inches (positive inches = left)
    private void strafeLeftInches(double inches, double power) {
        int moveCounts = (int) Math.round(inches * COUNTS_PER_INCH);

        // Reset encoders
        frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        // For mecanum strafing left: fl -> -move, fr -> +move, bl -> +move, br -> -move
        frontLeft.setTargetPosition(-moveCounts);
        frontRight.setTargetPosition(moveCounts);
        backLeft.setTargetPosition(moveCounts);
        backRight.setTargetPosition(-moveCounts);

        frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        double absPower = Math.abs(power);
        frontLeft.setPower(absPower);
        frontRight.setPower(absPower);
        backLeft.setPower(absPower);
        backRight.setPower(absPower);

        double timeoutS = Math.max(3.0, Math.abs(inches) / 12.0 * 1.0 + 3.0);
        ElapsedTime runtime = new ElapsedTime();

        while (opModeIsActive() && runtime.seconds() < timeoutS &&
                (frontLeft.isBusy() || frontRight.isBusy() || backLeft.isBusy() || backRight.isBusy())) {
            telemetry.addData("Strafing", "target=%d, fl=%d, fr=%d, bl=%d, br=%d", moveCounts,
                    frontLeft.getCurrentPosition(), frontRight.getCurrentPosition(), backLeft.getCurrentPosition(), backRight.getCurrentPosition());
            telemetry.update();
            sleep(20);
        }

        // stop
        frontLeft.setPower(0);
        frontRight.setPower(0);
        backLeft.setPower(0);
        backRight.setPower(0);

        frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }
}
