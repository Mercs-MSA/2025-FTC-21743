package org.firstinspires.ftc.teamcode;

import com.arcrobotics.ftclib.geometry.Pose2d;
import com.arcrobotics.ftclib.hardware.motors.Motor;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.arcrobotics.ftclib.hardware.motors.MotorEx;
//import com.arcrobotics.ftclib.hardware.motors.Encoder;

@TeleOp(name="jeffery2nd")
public class tutorialMecanum extends OpMode {
    private DcMotor frontLeft = null;
    private DcMotor frontRight = null;
    private DcMotor backLeft = null;
    private DcMotor backRight = null;
    private double speedMultiplier = 1.0;
    private boolean lastToggleState = false;
    private DcMotor outakeMotor = null;
    private CRServo leftoutakeServo = null;
    private CRServo rightoutakeServo = null;
    private MotorEx leftEncoder;
    private MotorEx rightEncoder;
    private MotorEx centerEncoder;

    //private OdometryTracker odometry = new OdometryTracker();





    public void initializeMotors() {
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


    public void initializeServos() {

        outakeMotor = hardwareMap.get(DcMotor.class, "outakeMotor");
        leftoutakeServo = hardwareMap.get(CRServo.class, "leftoutakeMotor");
        rightoutakeServo = hardwareMap.get(CRServo.class, "rightoutakeMotor");

    }



    @Override
    public void init() {
        initializeMotors();
        initializeServos();

        leftEncoder = new MotorEx(hardwareMap, "leftEncoder");
        rightEncoder = new MotorEx(hardwareMap, "rightEncoder");
        centerEncoder = new MotorEx(hardwareMap, "centerEncoder");

        leftEncoder.setRunMode(Motor.RunMode.RawPower);
        rightEncoder.setRunMode(Motor.RunMode.RawPower);
        centerEncoder.setRunMode(Motor.RunMode.RawPower);

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
        boolean currentToggleState = gamepad1.a;

        // Speed toggle logic
        if (currentToggleState && !lastToggleState) {
            speedMultiplier = (speedMultiplier == 1.0) ? 0.4 : 1.0;
        }
        lastToggleState = currentToggleState;


        mecanumDrive();
        double servoPower = gamepad2.right_stick_y;
        outakeMotor.setPower(servoPower);
        leftoutakeServo.setPower(servoPower);
        rightoutakeServo.setPower(servoPower);


        telemetry.addData("Speed Mode", speedMultiplier == 1.0 ? "Fast" : "Precision");
        telemetry.addData("Outake Power", servoPower);
        telemetry.addData("Front Left Power", frontLeft.getPower());
        telemetry.addData("Front Right Power", frontRight.getPower());
        telemetry.addData("Back Left Power", backLeft.getPower());
        telemetry.addData("Back Right Power", backRight.getPower());
        telemetry.addData("left Odometer Velocity", leftEncoder.getVelocity());
        telemetry.addData("right Odometer Velocity", rightEncoder.getVelocity());
        telemetry.addData("Left Encoder", leftEncoder.getCurrentPosition());
        telemetry.addData("Right Encoder", rightEncoder.getCurrentPosition());
        telemetry.addData("Center Encoder", centerEncoder.getCurrentPosition());
        //telemetry.addData("X (mm)", pose.getX());
        //telemetry.addData("Y (mm)", pose.getY());
        //telemetry.addData("Heading (deg)", Math.toDegrees(pose.getHEading()));
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