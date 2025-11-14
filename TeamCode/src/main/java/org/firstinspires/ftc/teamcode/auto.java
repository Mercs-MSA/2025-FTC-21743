package org.firstinspires.ftc.teamcode;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
//import com.arcrobotics.ftclib.hardware.motors.Encoder;


@Autonomous(name="Auto: Move Forward")
public class auto extends OpMode {
    private DcMotor frontLeft = null;
    private DcMotor frontRight = null;
    private DcMotor backLeft = null;
    private DcMotor backRight = null;
    private double speedMultiplier = 1.0;
    private boolean lastToggleState = false;
    private Servo gateServo = null;

    private ElapsedTime timer = new ElapsedTime();

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


    double ElapsedTime = 0;
    @Override
    public void init() {
        initializeDriveMotors();
        ElapsedTime = timer.seconds();

    }
    @Override
    public void loop() {
        while (ElapsedTime < 2.0) {
            frontLeft.setPower(0.2);
            backLeft.setPower(0.2);
            frontRight.setPower(0.2);
            backRight.setPower(0.2);

        }
    }
    }