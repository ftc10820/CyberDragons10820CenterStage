package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
@Autonomous
public class ScrimmageAutonomous extends LinearOpMode {

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

    int zone = 1;

    VisionSubsystem vision;

    @Override
    public void runOpMode() throws InterruptedException {

        initialize();
        vision = new VisionSubsystem(hardwareMap);
        vision.setAlliance("blue");

        while (!isStarted()) {
            vision.elementDetection(telemetry);
            vision.returnDistance(telemetry);
            telemetry.update();
        }
        zone = vision.zone;

        //waitForStart();

        if(opModeIsActive()) {

            if (zone == 1) {

                moveBackward(600, 0.75);
                turnLeft(500, 1.0);
                moveBackward(600, 0.75);
                intakeLeft.setPosition(1);
                sleep(500);

                moveBackward(1500, 0.75);

                craneAngle.setPosition(0.5);
                sleep(500);

                crane.setPower(-1);
                sleep(4500);
                crane.setPower(0);

                crane.setPower(1);
                sleep(2000);
                craneAngle.setPosition(0);
                sleep(2500);
                crane.setPower(0);



            } else if (zone == 2) {

                moveBackward(1100, 0.75);
                intakeLeft.setPosition(1);
                sleep(500);
                moveBackward(200, 0.75);
                turnLeft(425, 1.0);

                moveBackward(1750, 0.75);
                moveRight(1000, 0.75);
                craneAngle.setPosition(0.5);
                sleep(500);

                crane.setPower(-1);
                sleep(4500);
                crane.setPower(0);

                crane.setPower(1);
                sleep(2000);
                craneAngle.setPosition(0);
                sleep(2500);
                crane.setPower(0);


            } else {

                //Moves backwards towards zone
                moveBackward(700, 0.75);
                sleep(1000) ;
                //Turns left
                turnLeft(500, 1.0);
                sleep(1000) ;
                //Moves forward a little bit
                moveForward(200, 0.75);
                sleep(1000) ;

                //Drops off mosaic
                intakeLeft.setPosition(1);
                sleep(1000);

                moveBackward(200, 0.75) ;
                sleep(1000) ;
                turnRight(500, 1.0);
                sleep(1000) ;

                moveBackward(600, 0.75);
                sleep(1000) ;

                turnLeft(400, 1.0);
                sleep(1000) ;

                moveBackward(2000, 0.75);
                sleep(1000) ;

                turnRight(400, 1.0);
                sleep(1000) ;

                moveForward(500, 0.75);
                sleep(1000) ;

                turnLeft(500, 1.0);
                sleep(1000) ;

                craneAngle.setPosition(0.5);
                sleep(500);

                crane.setPower(-1);
                sleep(2500);
                crane.setPower(0);

                crane.setPower(1);
                sleep(1000);
                //craneAngle.setPosition(0);
                //sleep(1000);
                crane.setPower(0);
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
        intakeLeft.setPosition(0.3);
        intakeRight.setPosition(0.1);


        touchSensor = hardwareMap.get(TouchSensor.class, "Touch");

    }


    // time + speed are parameters for all the movement
    void moveBackward(int time, double speed) throws InterruptedException {
        frontLeft.setPower(speed);
        frontRight.setPower(speed);
        backLeft.setPower(speed);
        backRight.setPower(speed);

        Thread.sleep(time);

        frontLeft.setPower(0);
        frontRight.setPower(0);
        backLeft.setPower(0);
        backRight.setPower(0);

    }

    void moveForward(int time, double speed) throws InterruptedException {
        frontLeft.setPower(-speed);
        frontRight.setPower(-speed);
        backLeft.setPower(-speed);
        backRight.setPower(-speed);

        Thread.sleep(time);

        frontLeft.setPower(0);
        frontRight.setPower(0);
        backLeft.setPower(0);
        backRight.setPower(0);
    }

    void moveLeft(int time, double speed) throws InterruptedException {
        frontLeft.setPower(speed);
        frontRight.setPower(-speed);
        backLeft.setPower(-speed);
        backRight.setPower(speed);

        Thread.sleep(time);

        frontLeft.setPower(0);
        frontRight.setPower(0);
        backLeft.setPower(0);
        backRight.setPower(0);

    }

    void moveRight(int time, double speed) throws InterruptedException {
        frontLeft.setPower(-speed);
        frontRight.setPower(speed);
        backLeft.setPower(speed);
        backRight.setPower(-speed);

        Thread.sleep(time);

        frontLeft.setPower(0);
        frontRight.setPower(0);
        backLeft.setPower(0);
        backRight.setPower(0);
    }


    void turnRight(int time, double speed) throws InterruptedException {
        frontLeft.setPower(-speed);
        frontRight.setPower(speed);
        backLeft.setPower(-speed);
        backRight.setPower(speed);

        Thread.sleep(time);

        frontLeft.setPower(0);
        frontRight.setPower(0);
        backLeft.setPower(0);
        backRight.setPower(0);

    }

    void turnLeft(int time, double speed) throws InterruptedException {
        frontLeft.setPower(speed);
        frontRight.setPower(-speed);
        backLeft.setPower(speed);
        backRight.setPower(-speed);

        Thread.sleep(time);

        frontLeft.setPower(0);
        frontRight.setPower(0);
        backLeft.setPower(0);
        backRight.setPower(0);

    }

}
