package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.TouchSensor;

@TeleOp
public class CenterStageTeleOp extends LinearOpMode {

    public DcMotorEx frontLeft;
    public DcMotorEx frontRight;
    public DcMotorEx backLeft;
    public DcMotorEx backRight;

    public TouchSensor touchSensor;
    public DcMotorEx crane;
    public Servo indexer;
    public Servo intakeLeft;
    public Servo intakeRight;
    public Servo craneAngle;
    public Servo ramp;
    double speedVal = 1.0;

    boolean isTouched = false;

    @Override
    public void runOpMode() throws InterruptedException {

        initialize();
        waitForStart();

        while (opModeIsActive()) {

            // for the drive train
            double y = gamepad1.left_stick_y;
            double x = gamepad1.left_stick_x * -1.1;
            double rx = -gamepad1.right_stick_x * 0.75;

            double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
            double frontLeftPower = (y + x + rx) / denominator;
            double backLeftPower = (y - x + rx) / denominator;
            double frontRightPower = (y - x - rx) / denominator;
            double backRightPower = (y + x - rx) / denominator;



            frontLeft.setPower(frontLeftPower * speedVal);
            backLeft.setPower(backLeftPower * speedVal);
            frontRight.setPower(frontRightPower * speedVal);
            backRight.setPower(backRightPower * speedVal);
            telemetry.addData("Is Pressed", touchSensor.isPressed());
            telemetry.update();
            //Sees if the controller is going backwards and if the touch sensor is being touched and the disables it from moving backwards
            if (touchSensor.isPressed()){
                isTouched = true;
                crane.setPower(0);
            } else if (gamepad2.left_stick_y < 0) {
                isTouched = false;
            }
            if(isTouched == false) {
                crane.setPower(gamepad2.left_stick_y);
            }

            if (gamepad2.y) {

                craneAngle.setPosition(1);

            } else if (gamepad2.a) {

                craneAngle.setPosition(0);

            } else if (gamepad2.x) {

                craneAngle.setPosition(0.5);

            }

            if (gamepad2.dpad_down) {
                frontLeft.setPower(-.5);
                backLeft.setPower(-.5);
                frontRight.setPower(-.5);
                backRight.setPower(-.5);

                sleep(500);

                frontLeft.setPower(0);
                backLeft.setPower(0);
                frontRight.setPower(0);
                backRight.setPower(0);

                craneAngle.setPosition(0);
                crane.setPower(1);
                while(!touchSensor.isPressed()){}
                crane.setPower(0);

            }



            /*
            if (gamepad1.y) {

                indexer.setPosition(0.6);

            } else if (gamepad1.a) {

                indexer.setPosition(0);

            }

             */

            if (gamepad1.y) {

                intakeLeft.setPosition(0.2);
                intakeRight.setPosition(0.9);

            } else if (gamepad1.a) {

                intakeLeft.setPosition(0.9);
                intakeRight.setPosition(0.1);

            }







            /*
            if (gamepad1.y) {

                ramp.setPosition(0.35);

            } else if (gamepad1.a) {

                ramp.setPosition(0.45);

            }

             */

            if (gamepad1.x) {

                ramp.setPosition(0.45);
                indexer.setPosition(0);
                sleep(500);

                intakeLeft.setPosition(0);
                intakeRight.setPosition(0.9);
                sleep(500);

                indexer.setPosition(1);
                sleep(1000);

                ramp.setPosition(0.3);
                indexer.setPosition(0.25);
                intakeLeft.setPosition(1);
                intakeRight.setPosition(0.1);
            }





        }

    }

    public void initialize() {


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

        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        //crane motor
        crane = hardwareMap.get(DcMotorEx.class, "Crane");

        //servos
        indexer = hardwareMap.get(Servo.class, "Indexer");
        intakeLeft = hardwareMap.get(Servo.class, "IntakeLeft");
        intakeRight = hardwareMap.get(Servo.class, "IntakeRight");
        craneAngle = hardwareMap.get(Servo.class, "CraneAngle");
        ramp = hardwareMap.get(Servo.class, "Ramp");


        ramp.setPosition(0.3);
        indexer.setPosition(0.25);
        intakeLeft.setPosition(1);
        intakeRight.setPosition(0.1);


        touchSensor = hardwareMap.get(TouchSensor.class, "Touch");



    }
}
