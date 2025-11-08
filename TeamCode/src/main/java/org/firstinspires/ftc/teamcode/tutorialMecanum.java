package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;


import org.firstinspires.ftc.robotcore.internal.usb.EthernetOverUsbSerialNumber;
//import com.arcrobotics.ftclib.hardware.motors.Encoder;

@TeleOp(name="jeffery2nd")
public class tutorialMecanum extends OpMode {
    private DcMotor frontLeft = null;
    private DcMotor frontRight = null;
    private DcMotor backLeft = null;
    private DcMotor backRight = null;
    private double speedMultiplier = 1.0;
    private boolean lastToggleState = false;
    private DcMotor leftoutakeMotor = null;
    private DcMotor rightoutakeMotor = null;
    //private CRServo leftoutakeServo = null;
    //private CRServo rightoutakeServo = null;
    private boolean intakeReverse = false;
    private boolean lastIntakeToggle = false;
    //private MotorEx leftEncoder;
    //private MotorEx rightEncoder;
    //private MotorEx centerEncoder;

    //private OdometryTracker odometry = new OdometryTracker();

    private DcMotor intakeMotor = null;
    private Servo gateServo = null;
    private Servo leftAimServo = null;
    private Servo rightAimServo = null;




    public void initializeDriveMotors() {
        frontLeft = hardwareMap.get(DcMotor.class, "frontLeft");
        frontRight = hardwareMap.get(DcMotor.class, "frontRight");
        backLeft = hardwareMap.get(DcMotor.class, "backLeft");
        backRight = hardwareMap.get(DcMotor.class, "backRight");

        frontLeft.setDirection(DcMotor.Direction.REVERSE);
        backLeft.setDirection(DcMotor.Direction.REVERSE);

        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);


    }


    public void initializeShooter() {

        leftoutakeMotor = hardwareMap.get(DcMotor.class, "leftoutakeMotor");
        rightoutakeMotor = hardwareMap.get(DcMotor.class, "rightoutakeMotor");
        leftoutakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rightoutakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        rightoutakeMotor.setDirection(DcMotor.Direction.REVERSE);

        intakeMotor = hardwareMap.get(DcMotor.class, "intakeMotor");

        gateServo = hardwareMap.get(Servo.class, "gateServo");
        //gateServo.setDirection(Servo.Direction.REVERSE);

        leftAimServo = hardwareMap.get(Servo.class, "leftAimServo");
        rightAimServo = hardwareMap.get(Servo.class, "rightAimServo");
        leftAimServo.scaleRange(0.0,0.6);
        rightAimServo.scaleRange(0.0,0.6);
        rightAimServo.setDirection(Servo.Direction.REVERSE);

    }

    @Override
    public void init() {
        initializeDriveMotors();
        initializeShooter();

        //leftEncoder = new MotorEx(hardwareMap, "leftEncoder");
        //rightEncoder = new MotorEx(hardwareMap, "rightEncoder");
        //centerEncoder = new MotorEx(hardwareMap, "centerEncoder");

        //leftEncoder.setRunMode(Motor.RunMode.RawPower);
        //rightEncoder.setRunMode(Motor.RunMode.RawPower);
        //centerEncoder.setRunMode(Motor.RunMode.RawPower);

//        leftEncoder.setRunMode(Motor.RunMode.STOP_AND_RESET_ENCODER);
//        leftEncoder.setRunMode(Motor.RunMode.RUN_WITHOUT_ENCODER);
//        rightEncoder.setRunMode(Motor.RunMode.STOP_AND_RESET_ENCODER);
//        rightEncoder.setRunMode(Motor.RunMode.RUN_WITHOUT_ENCODER);
//        centerEncoder.setRunMode(Motor.RunMode.STOP_AND_RESET_ENCODER);
//        centerEncoder.setRunMode(Motor.RunMode.RUN_WITHOUT_ENCODER);

        //leftEncoder = new Encoder(hardwareMap, "leftEncoder");
        //rightEncoder = new Encoder(hardwareMap, "rightEncoder");
        //centerEncoder = new Encoder(hardwareMap, "centerEncoder");

    }



    @Override
    public void loop() {
        boolean precisionMode = gamepad1.a;
        double outakePower = 0.0;
        boolean gateUp;
        boolean currentToggle = gamepad2.right_bumper;

        // Speed toggle logic
        if (precisionMode && !lastToggleState) {
            speedMultiplier = (speedMultiplier == 1.0) ? 0.4 : 1.0;
        }
        lastToggleState = precisionMode;

        if (gamepad2.b) {
            outakePower = 0.8;
        } else if (gamepad2.a) {
            outakePower = 0.75;
        } else if (gamepad2.x) {
            outakePower = 0.7;
        }

        if (gamepad2.y) {
            outakePower = (-0.1);
        }

        leftoutakeMotor.setPower(outakePower);
        rightoutakeMotor.setPower(outakePower);

        if (gamepad2.left_bumper) {
            intakeMotor.setPower(1.0);
        } else if (gamepad2.right_bumper) {
            intakeMotor.setPower(-1.0);
        } else {
            intakeMotor.setPower(0.0);
        }

        if (currentToggle && !lastIntakeToggle) {
            intakeReverse = !intakeReverse;
        }
        lastIntakeToggle = currentToggle;

        mecanumDrive();

        gateUp = gamepad2.dpad_down;
        if (gateUp) {
            gateServo.setPosition(0.0);
        } else
        {
            gateServo.setPosition(0.25);
        }
//        gateServo.setPosition(gateUp ? 0.5 : 0.0);
//        //leftoutakeServo.setPower(servoPower);
//        //rightoutakeServo.setPower(servoPower);

        float aimPosition = gamepad2.left_trigger;
        leftAimServo.setPosition(aimPosition);
        rightAimServo.setPosition(aimPosition);

        telemetry.addData("Speed Mode", speedMultiplier == 1.0 ? "Fast" : "Precision");
        telemetry.addData("Outake Power", outakePower);
        telemetry.addData("Front Left Power", frontLeft.getPower());
        telemetry.addData("Front Right Power", frontRight.getPower());
        telemetry.addData("Back Left Power", backLeft.getPower());
        telemetry.addData("Back Right Power", backRight.getPower());
        telemetry.addData("Left Outake Motor Power", leftoutakeMotor.getPower());
        telemetry.addData("Right Outake Motor Power", rightoutakeMotor.getPower());
        telemetry.addData("Intake Motor Power", intakeMotor.getPower());
        telemetry.addData("Gate", gateUp ? "Up" : "Down");
        telemetry.addData("Aim Position", aimPosition);

        //telemetry.addData("left Odometer Velocity", leftEncoder.getVelocity());
        //telemetry.addData("right Odometer Velocity", rightEncoder.getVelocity());
        //telemetry.addData("Left Encoder", leftEncoder.getCurrentPosition());
        //telemetry.addData("Right Encoder", rightEncoder.getCurrentPosition());
        //telemetry.addData("Center Encoder", centerEncoder.getCurrentPosition());
        //telemetry.addData("X (mm)", pose.getX());
        //telemetry.addData("Y (mm)", pose.getY());
        //telemetry.addData("Heading (deg)", Math.toDegrees(pose.getHEading()));
        //telemetry.addData( "")
        telemetry.update();

        //odometry.update(
        //        leftEncoder.getCurrentPosition(),
        //        rightEncoder.getCurrentPosition(),
        //        centerEncoder.getCurrentPosition()
        //);

        //Pose2d pose = odometry.getPose();
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

        double max = Math.max(Math.abs(frontLeftPower),
                Math.max(Math.abs(frontRightPower),
                        Math.max(Math.abs(backLeftPower), Math.abs(backRightPower))));
        if (max > 1.0) {
            frontLeftPower /= max;
            frontRightPower /= max;
            backLeftPower /= max;
            backRightPower /= max;
        }


        frontLeft.setPower(frontLeftPower * speedMultiplier);
        frontRight.setPower(frontRightPower * speedMultiplier);
        backLeft.setPower(backLeftPower * speedMultiplier);
        backRight.setPower(backRightPower * speedMultiplier);


    }}