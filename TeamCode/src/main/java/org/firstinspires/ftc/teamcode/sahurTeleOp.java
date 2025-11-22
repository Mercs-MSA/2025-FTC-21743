package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
//import com.arcrobotics.ftclib.hardware.motors.Encoder;

@TeleOp(name="sahurTeleOp")
public class sahurTeleOp extends OpMode {
    private DcMotor frontLeft = null;
    private DcMotor frontRight = null;
    private DcMotor backLeft = null;
    private DcMotor backRight = null;
    private double speedMultiplier = 1.0;
    private boolean lastToggleState = false;
    private Servo gateServo = null;
//    private DcMotor outakeMotor = null;
//    private CRServo leftoutakeServo = null;
//    private CRServo rightoutakeServo = null;
    //private MotorEx leftEncoder;
    //private MotorEx rightEncoder;
    //private MotorEx centerEncoder;

    //private OdometryTracker odometry = new OdometryTracker();

    GoBildaPinpointDriver pinpoint;


    public void configurePinpoint(){
        /*
         *  Set the odometry pod positions relative to the point that you want the position to be measured from.
         *
         *  The X pod offset refers to how far sideways from the tracking point the X (forward) odometry pod is.
         *  Left of the center is a positive number, right of center is a negative number.
         *
         *  The Y pod offset refers to how far forwards from the tracking point the Y (strafe) odometry pod is.
         *  Forward of center is a positive number, backwards is a negative number.
         */
        pinpoint.setOffsets(-84.0, -168.0, DistanceUnit.MM); //these are tuned for 3110-0002-0001 Product Insight #1

        /*
         * Set the kind of pods used by your robot. If you're using goBILDA odometry pods, select either
         * the goBILDA_SWINGARM_POD, or the goBILDA_4_BAR_POD.
         * If you're using another kind of odometry pod, uncomment setEncoderResolution and input the
         * number of ticks per unit of your odometry pod.  For example:
         *     pinpoint.setEncoderResolution(13.26291192, DistanceUnit.MM);
         */
        pinpoint.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);

        /*
         * Set the direction that each of the two odometry pods count. The X (forward) pod should
         * increase when you move the robot forward. And the Y (strafe) pod should increase when
         * you move the robot to the left.
         */
        pinpoint.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD,
                GoBildaPinpointDriver.EncoderDirection.FORWARD);

        /*
         * Before running the robot, recalibrate the IMU. This needs to happen when the robot is stationary
         * The IMU will automatically calibrate when first powered on, but recalibrating before running
         * the robot is a good idea to ensure that the calibration is "good".
         * resetPosAndIMU will reset the position to 0,0,0 and also recalibrate the IMU.
         * This is recommended before you run your autonomous, as a bad initial calibration can cause
         * an incorrect starting value for x, y, and heading.
         */
        pinpoint.resetPosAndIMU();
    }

    public void initPinPoint() {
        // Get a reference to the sensor
        pinpoint = hardwareMap.get(GoBildaPinpointDriver.class, "odoComp");

        // Configure the sensor
        configurePinpoint();

        // Set the location of the robot - this should be the place you are starting the robot from
        pinpoint.setPosition(new Pose2D(DistanceUnit.INCH, 0, 0, AngleUnit.DEGREES, 0));
    }

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

        gateServo = hardwareMap.get(Servo.class, "gateServo");


    }


//    public void initializeShooter() {
//
//        outakeMotor = hardwareMap.get(DcMotor.class, "outakeMotor");
//        leftoutakeServo = hardwareMap.get(CRServo.class, "leftoutakeMotor");
//        rightoutakeServo = hardwareMap.get(CRServo.class, "rightoutakeMotor");
//
//    }



    @Override
    public void init() {
        initializeDriveMotors();
        initPinPoint();
//        initializeShooter();

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
        boolean currentToggleState = gamepad1.a;

        // Speed toggle logic
        if (currentToggleState && !lastToggleState) {
            speedMultiplier = (speedMultiplier == 1.0) ? 0.4 : 1.0;
        }
        lastToggleState = currentToggleState;
        telemetry.addLine("Push your robot around to see it track");
        telemetry.addLine("Press A to reset the position");
        if(gamepad1.b){
            // You could use readings from April Tags here to give a new known position to the pinpoint
            pinpoint.setPosition(new Pose2D(DistanceUnit.INCH, 0, 0, AngleUnit.DEGREES, 0));
        }
        pinpoint.update();
        Pose2D pose2D = pinpoint.getPosition();

        telemetry.addData("X coordinate (IN)", pose2D.getX(DistanceUnit.INCH));
        telemetry.addData("Y coordinate (IN)", pose2D.getY(DistanceUnit.INCH));
        telemetry.addData("Heading angle (DEGREES)", pose2D.getHeading(AngleUnit.DEGREES));

        mecanumDrive();
        double servoPower = gamepad2.right_stick_y;
//        outakeMotor.setPower(servoPower);
//        leftoutakeServo.setPower(servoPower);
//        rightoutakeServo.setPower(servoPower);


        if (gamepad2.dpad_down) {
            gateServo.setPosition(0.0);
        } else
        {
            gateServo.setPosition(0.25);
        }

        telemetry.addData("Speed Mode", speedMultiplier == 1.0 ? "Fast" : "Precision");
        telemetry.addData("Outake Power", servoPower);
        telemetry.addData("Front Left Power", frontLeft.getPower());
        telemetry.addData("Front Right Power", frontRight.getPower());
        telemetry.addData("Back Left Power", backLeft.getPower());
        telemetry.addData("Back Right Power", backRight.getPower());
        //telemetry.addData("left Odometer Velocity", leftEncoder.getVelocity());
        //telemetry.addData("right Odometer Velocity", rightEncoder.getVelocity());
        //telemetry.addData("Left Encoder", leftEncoder.getCurrentPosition());
        //telemetry.addData("Right Encoder", rightEncoder.getCurrentPosition());
        //telemetry.addData("Center Encoder", centerEncoder.getCurrentPosition());
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