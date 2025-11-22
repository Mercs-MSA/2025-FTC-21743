package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

@Autonomous(name = "Encoder Auto: Swyft Drive + REV Odometry", group = "Linear Opmode")
public class AutoBlueLeft extends LinearOpMode {

    private DcMotor frontLeft, frontRight, backLeft, backRight;

    static final double COUNTS_PER_MOTOR_REV = 537.6; // GoBilda 5202 or similar
    static final double DRIVE_GEAR_REDUCTION = 1.0;
    static final double WHEEL_DIAMETER_INCHES = 3.78; // Swyft wheels
    static final double COUNTS_PER_INCH = (COUNTS_PER_MOTOR_REV * DRIVE_GEAR_REDUCTION) /
            (WHEEL_DIAMETER_INCHES * Math.PI);

    @Override
    public void runOpMode() {
        frontLeft  = hardwareMap.get(DcMotor.class, "frontLeft");
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        backLeft   = hardwareMap.get(DcMotor.class, "backLeft");
        backRight  = hardwareMap.get(DcMotor.class, "backRight");

        frontRight.setDirection(DcMotor.Direction.REVERSE);
        backRight.setDirection(DcMotor.Direction.REVERSE);

        resetEncoders();

        waitForStart();

        if (opModeIsActive()) {
            driveInches(36, 0.5);
            turnDegrees(135, 0.4);
            driveInches(24, 0.5);
            driveInches(-24, 0.5);
            turnDegrees(45, 0.4);
        }
    }

    private void resetEncoders() {
        frontLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        frontRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backLeft.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        backRight.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    private void driveInches(double inches, double speed) {
        int moveCounts = (int)(inches * COUNTS_PER_INCH);

        frontLeft.setTargetPosition(frontLeft.getCurrentPosition() + moveCounts);
        frontRight.setTargetPosition(frontRight.getCurrentPosition() + moveCounts);
        backLeft.setTargetPosition(backLeft.getCurrentPosition() + moveCounts);
        backRight.setTargetPosition(backRight.getCurrentPosition() + moveCounts);

        setRunToPosition();
        setAllPower(speed);

        while (opModeIsActive() && motorsBusy()) {
            telemetry.addData("Driving", "%.1f inches", inches);
            telemetry.update();
        }

        stopAll();
        setRunUsingEncoder();
    }

    private void turnDegrees(double degrees, double speed) {
        // Approximate: 1 degree ≈ X encoder ticks per side
        double TURN_DIAMETER_INCHES = 16.0; // adjust based on your robot's turning diameter
        double turnCircumference = Math.PI * TURN_DIAMETER_INCHES;
        double turnDistance = (degrees / 360.0) * turnCircumference;
        int turnCounts = (int)(turnDistance * COUNTS_PER_INCH);

        frontLeft.setTargetPosition(frontLeft.getCurrentPosition() - turnCounts);
        backLeft.setTargetPosition(backLeft.getCurrentPosition() - turnCounts);
        frontRight.setTargetPosition(frontRight.getCurrentPosition() + turnCounts);
        backRight.setTargetPosition(backRight.getCurrentPosition() + turnCounts);

        setRunToPosition();
        setAllPower(speed);

        while (opModeIsActive() && motorsBusy()) {
            telemetry.addData("Turning", "%.1f degrees", degrees);
            telemetry.update();
        }

        stopAll();
        setRunUsingEncoder();
    }

    private void setRunToPosition() {
        frontLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        frontRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backLeft.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        backRight.setMode(DcMotor.RunMode.RUN_TO_POSITION);
    }

    private void setRunUsingEncoder() {
        frontLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        frontRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backLeft.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        backRight.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    private void setAllPower(double power) {
        frontLeft.setPower(power);
        frontRight.setPower(power);
        backLeft.setPower(power);
        backRight.setPower(power);
    }

    private void stopAll() {
        setAllPower(0);
    }

    private boolean motorsBusy() {
        return frontLeft.isBusy() || frontRight.isBusy() || backLeft.isBusy() || backRight.isBusy();
    }
}