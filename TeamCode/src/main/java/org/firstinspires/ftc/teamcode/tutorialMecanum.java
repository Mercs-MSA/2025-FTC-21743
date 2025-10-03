package org.firstinspires.ftc.teamcode;

import static org.firstinspires.ftc.robotcore.external.BlocksOpModeCompanion.gamepad1;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;

@TeleOp(name="jeffery2nd")
public class tutorialMecanum extends OpMode {
    private DcMotor frontLeft = null;
    private DcMotor frontRight = null;
    private DcMotor backLeft = null;
    private DcMotor backRight = null;
    private double speedMultiplier = 1.0;
    private boolean lastToggleState = false;
    private DcMotor outakeMotor = null;

    public void initializeMotors() {
        frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        backLeft = hardwareMap.get(DcMotor.class, "backLeft");
        backRight = hardwareMap.get(DcMotor.class, "backRight");

        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        backLeft.setDirection(DcMotor.Direction.REVERSE);
        //outakeServo.setDirection(CRServo.Direction.REVERSE);

        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }


    public void initializeServos() {

        outakeMotor = hardwareMap.get(DcMotor.class, "outakeMotor");

    }



    @Override
    public void init() {
        initializeMotors();
        initializeServos();

    }




    @Override
    public void loop() {
        boolean currentToggleState = gamepad1.a;

        if (currentToggleState && !lastToggleState) {
            speedMultiplier = (speedMultiplier == 1.0) ? 0.4: 1.0;
        }
        lastToggleState = currentToggleState;

        mecanumDrive();

        double servoPower = gamepad2.right_stick_y;
        outakeMotor.setPower(servoPower);

    }
    public void buttonTester () {
        if (gamepad1.a) {
            frontLeft.setPower(.5);
        } else {
            frontLeft.setPower(0);
        }

    }
    public void trigTester() {
        double trigPower = gamepad1.right_trigger;
        frontLeft.setPower(trigPower);
    }

    public void mecanumDrive() {
        double axial = -gamepad1.left_stick_y;
        double lateral = gamepad1.left_stick_x;
        double yaw = gamepad1.right_stick_x;

        double frontLeftPower = axial + lateral + yaw;
        double frontRightPower = axial - lateral - yaw;
        double backLeftPower = axial - lateral + yaw;
        double backRightPower = axial + lateral - yaw;

        // double max = Math.max(Math.abs(frontLeftPower),
        //              Math.max(Math.abs(frontRightPower),
        //              Math.max(Math.abs(backLeftPower), Math.abs(backRightPower))));
        // if (max > 1.0) {
        //     frontLeftPower /= max;
        //     frontRightPower /= max;
        //     backLeftPower /= max;
        //     backRightPower /= max;
        // }


        frontLeft.setPower(frontLeftPower * speedMultiplier);
        frontRight.setPower(frontRightPower * speedMultiplier);
        backLeft.setPower(backLeftPower * speedMultiplier);
        backRight.setPower(backRightPower * speedMultiplier);


    }}