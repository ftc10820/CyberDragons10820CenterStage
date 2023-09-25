package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

@TeleOp
public class MecanumChassis extends LinearOpMode {

    public DcMotorEx frontLeft;
    public DcMotorEx frontRight;
    public DcMotorEx backLeft;
    public DcMotorEx backRight;

    public DcMotorEx linearSlide;
    public Servo grabber;
    public CRServo grabberTurner;
    public Servo shooter;
    public CRServo transfer;

    @Override
    public void runOpMode() throws InterruptedException {

        initialize();

        waitForStart();
        while (opModeIsActive()) {

            // for the drive train
            double y = gamepad1.left_stick_y; // Remember, this is reversed!
            double x = gamepad1.left_stick_x * -1.1; // Counteract imperfect strafing
            double rx = -gamepad1.right_stick_x * 0.75;

            // Denominator is the largest motor power (absolute value) or 1
            // This ensures all the powers maintain the same ratio, but only when
            // at least one is out of  the range [-1, 1]


            // Denominator is the largest motor power (absolute value) or 1
            // This ensures all the powers maintain the same ratio, but only when
            // at least one is out of the range [-1, 1]
            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
            double frontLeftPower = (y + x + rx) / denominator;
            double backLeftPower = (y - x + rx) / denominator;
            double frontRightPower = (y - x - rx) / denominator;
            double backRightPower = (y + x - rx) / denominator;

            frontLeft.setPower(frontLeftPower);
            backLeft.setPower(backLeftPower);
            frontRight.setPower(frontRightPower);
            backRight.setPower(backRightPower);

            linearSlide.setPower(gamepad2.right_stick_y * 0.75);

            transfer.setPower(gamepad2.left_stick_y);

            if (gamepad2.y) {

                grabber.setPosition(1);
            } else if (gamepad2.a) {

                grabber.setPosition(0);
            }

            if (gamepad1.y) {

                shooter.setPosition(1);
            }

            if (gamepad2.dpad_up) {

                grabberTurner.setPower(1);
            } else if (gamepad2.dpad_down) {

                grabberTurner.setPower(-1);
            }




        }

    }

    void initialize() {
        // setting up drive train
        frontRight = hardwareMap.get(DcMotorEx.class, "FrontRight");
        frontLeft = hardwareMap.get(DcMotorEx.class, "FrontLeft");
        backRight = hardwareMap.get(DcMotorEx.class, "BackRight");
        backLeft = hardwareMap.get(DcMotorEx.class, "BackLeft");

        frontRight.setPower(0);
        frontLeft.setPower(0);
        backRight.setPower(0);
        backLeft.setPower(0);

        frontRight.setDirection(DcMotorEx.Direction.REVERSE);
        backRight.setDirection(DcMotorEx.Direction.REVERSE);

        linearSlide = hardwareMap.get(DcMotorEx.class, "LinearSlide");

        grabber = hardwareMap.get(Servo.class, "Grabber");
        grabberTurner = hardwareMap.get(CRServo.class, "GrabberTurner");
        transfer = hardwareMap.get(CRServo.class, "Transfer");
        shooter = hardwareMap.get(Servo.class, "Shooter");


    }
}
