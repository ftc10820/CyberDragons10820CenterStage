package org.firstinspires.ftc.teamcode;

import android.util.Size;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.vision.tfod.TfodProcessor;

import java.util.List;
import java.util.concurrent.TimeUnit;

@TeleOp
public class States_TeleOp extends LinearOpMode {
    // use the routines that are present in initRobotTest
    // and ensure the same functions are used in Autonomous to reduce confusion

    // drive train motors
    public DcMotorEx frontLeft;
    public DcMotorEx frontRight;
    public DcMotorEx backLeft;
    public DcMotorEx backRight;

    // there are specific ways that the drive power is calculated based on automations
    double frontLeftPower = 0.0, backLeftPower = 0.0, frontRightPower = 0.0, backRightPower = 0.0 ;

    // crane linear slide and lifter
    public DcMotorEx crane;
    public Servo craneAngle;

    // suspension motor
    public DcMotorEx suspension;

    // the rotating platform for intake
    public Servo indexer;

    // The servo used for intake (left and right - with the robot's perspective)
    public Servo intakeLeft;
    public Servo intakeRight;

    // the short ramp used for the intake mechanism
    public Servo ramp;

    // the drone launcher
    public Servo droneLauncher;

    // DCMotors being used to drive LEDs
    public DcMotorSimple ledRight ;
    public DcMotorSimple ledLeft ;

    // sensors used for pixel intake
    private ColorSensor pixelDetectorRight;
    private ColorSensor pixelDetectorLeft;

    // distance sensor on the bucket to detect when bucket is close to backdrop
    private DistanceSensor distanceBucket;

    // touch sensor used with linear slide
    public TouchSensor touchCrane;
    // Color sensor at the back of the robot used for detecting lines on the field
    // NOTE: Reusing for bucket, but needs to renamed
    public ColorSensor colorFieldLine;


    // helper variables
    ElapsedTime eTime1 = new ElapsedTime() ;
    ElapsedTime eTime2 = new ElapsedTime() ;

    ElapsedTime eTeleOp = new ElapsedTime() ;

    double speedFactor = 0.75;

    int craneMax = 4500;

    double cranePosUp = 0.0 ;
    double cranePosDown = 1.0 ;
    double curCranePos = 0.0 ;

    boolean craneUpFlag = false ;
    boolean craneDownFlag = false ;

    public void runOpMode() throws InterruptedException {

        initialize(); // this is only for initializing the drives, servos and sensors

        initAutonomous(); // initialize servos for autonomous ; raise ramp and open intake
        telemetry.addData("encoder value: ", crane.getCurrentPosition());
        telemetry.addData("Initialized.", "Press START..");
        telemetry.update();

        waitForStart();

        cranePosUp = craneAngle.getPosition() ;
        cranePosDown = craneAngle.getPosition() ;

        eTeleOp.reset();

        while (opModeIsActive()) {

            telemetry.addData("crane encoder value: ", crane.getCurrentPosition());
            telemetry.addData("right color sensor value: ", getPixelDetectionRightVal());
            telemetry.addData("left color sensor value: ", getPixelDetectionLeftVal());
            //telemetry.addData("bucket distance: ", distanceBucket.getDistance(DistanceUnit.CM));
            telemetry.addData("bucket: ", "alpha " + colorFieldLine.alpha() + " red " + colorFieldLine.red() + " green " + colorFieldLine.green() + " blue " + colorFieldLine.blue() + " hue " + colorFieldLine.argb());
            telemetry.update();

            // light up the LEDs
            if(isPixelDetectedLeft()) {
                ledLeft.setPower(1.0);
            } else {
                ledLeft.setPower(0.0);
            }

            // light up the LEDs
            if(isPixelDetectedRight()) {
                ledRight.setPower(1.0);
            } else {
                ledRight.setPower(0.0);
            }

            curCranePos = craneAngle.getPosition() ;

            // Run the drive motors
            frontLeft.setPower(frontLeftPower);
            frontRight.setPower(frontRightPower);
            backLeft.setPower(backLeftPower);
            backRight.setPower(backRightPower);

            // individual controls for different sub-systems
            if (gamepad1.a) {
                intakePixel();  // does not throw pixel into bucket
            }

            if (gamepad1.y) {
                int retval = extendCraneUseColorSensorVelocity(4000, 5000, 600, 3000);
                //sleep(200) ;
                if (retval == 0) {
                    liftCraneSlightly(0.2);
                    sleep(250) ;
                    retractCraneHome(0.8, 2000);
                    positionCraneBase();
                    retractCraneHome(0.8, 2000);
                }
            }

            if (gamepad1.x) {
                retractCraneHome(0.8, 2500);
            }

            if (gamepad2.y) {
                positionCraneMedium();
            } else if (gamepad2.a) {
                positionCraneBase();
            } else if (gamepad2.x) {
                positionCraneLow();
            } else if (gamepad2.b)
                positionCraneHigh();

            // take distance sensor and touch sensor into account
            // logic for manual control of linear slide
            double cranePower = -gamepad2.left_stick_y;
            /*if ((cranePower > 0) && (distanceBucket.getDistance(DistanceUnit.CM) < 15))
                cranePower *= 0.2;
            if ((cranePower < 0) && (touchCrane.isPressed()))
                cranePower *= 0.1;*/
            extendCraneVelocity(cranePower*4000);

            // logic for manual lifting of crane
            /*
            if (gamepad2.dpad_left) {
                if (craneUpFlag == false) {
                    craneDownFlag = false ;
                    craneUpFlag = true ;
                    cranePosUp = craneAngle.getPosition() + 0.1 ;
                    if (cranePosUp > 1.0)
                        cranePosUp = 1.0;
                    craneAngle.setPosition(cranePosUp);
                } else {
                    if (craneAngle.getPosition() > (cranePosUp - 0.02)) {
                        cranePosUp += 0.1;
                        if (cranePosUp > 1.0)
                            cranePosUp = 1.0;
                        craneAngle.setPosition(cranePosUp);
                    }
                }
            }

            if (gamepad2.dpad_right) {
                if (craneDownFlag == false) {
                    craneUpFlag = false ;
                    craneDownFlag = true ;
                    cranePosDown = craneAngle.getPosition() - 0.1 ;
                    if (cranePosDown < 0.0)
                        cranePosDown = 0.0;
                    craneAngle.setPosition(cranePosDown);
                } else {
                    if (craneAngle.getPosition() < (cranePosDown + 0.02)) {
                        cranePosDown -= 0.1;
                        if (cranePosDown < 0.0)
                            cranePosDown = 0.0;
                        craneAngle.setPosition(cranePosDown);
                    }
                }
            }
             */

            // only for end game
            //if(eTeleOp.milliseconds() > 90000) {
            driveSuspension(-gamepad2.right_stick_y);

            if (gamepad2.right_trigger > 0) {
                releaseDrone();
            }

            if (gamepad2.left_trigger > 0) {
                releaseSuspensionHook();
                extendCraneVelocity(4000);
                sleep(100);
                stopCrane();
            }
            //}

            if(gamepad2.dpad_up) { // throws pixel into bucket
                liftIntakePlatform();
            }

            if(gamepad2.dpad_down) {
                initIntakePlatform();
            }

        }

    }


    // TODO: Refactor initialize() to RobotClass. OpMode.initialize() should be robot = new RobotClass(); robot.initialize();
    public void initialize() {


        // setting up drive train
        frontRight = hardwareMap.get(DcMotorEx.class, "FrontRight");
        frontLeft = hardwareMap.get(DcMotorEx.class, "FrontLeft");
        backRight = hardwareMap.get(DcMotorEx.class, "BackRight");
        backLeft = hardwareMap.get(DcMotorEx.class, "BackLeft");

        // this ensures that all wheels go forward when applying positive power
        frontRight.setDirection(DcMotorEx.Direction.FORWARD);
        backRight.setDirection(DcMotorEx.Direction.FORWARD);
        frontLeft.setDirection(DcMotorEx.Direction.REVERSE);
        backLeft.setDirection(DcMotorEx.Direction.REVERSE);

        frontRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        frontLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backRight.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        backLeft.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        //crane motor

        crane = hardwareMap.get(DcMotorEx.class, "Crane");
        crane.setDirection(DcMotorEx.Direction.REVERSE);
        crane.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        crane.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        crane.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        //suspension motor
        suspension = hardwareMap.get(DcMotorEx.class, "Suspension");
        suspension.setDirection(DcMotorEx.Direction.REVERSE);
        suspension.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        //servos
        indexer = hardwareMap.get(Servo.class, "Indexer");
        intakeLeft = hardwareMap.get(Servo.class, "IntakeLeft");
        intakeRight = hardwareMap.get(Servo.class, "IntakeRight");
        craneAngle = hardwareMap.get(Servo.class, "CraneAngle");
        ramp = hardwareMap.get(Servo.class, "Ramp");
        droneLauncher = hardwareMap.get(Servo.class, "DroneLauncher");

        // Drivers being used for LEDs
        ledRight = hardwareMap.get(DcMotorSimple.class, "LedRight");
        ledLeft = hardwareMap.get(DcMotorSimple.class, "LedLeft");

        //sensors
        touchCrane = hardwareMap.get(TouchSensor.class, "Touch");
        colorFieldLine = hardwareMap.get(ColorSensor.class, "Color");

        pixelDetectorLeft = hardwareMap.get(ColorSensor.class, "PixelDetectorLeft");
        pixelDetectorRight = hardwareMap.get(ColorSensor.class, "PixelDetectorRight");

        distanceBucket = hardwareMap.get(DistanceSensor.class, "DistanceBucket") ;
    }

    void initAutonomous() {
        liftRamp();
        initIntakePlatform();
        openRightIntake();
        openLeftIntake();

        initDroneLauncher();
    }


    // TODO: Refactor moveBackward(time, speed) to RobotClass and update calls in this OpMode
    // time + speed are parameters for all the movement
    void moveBackward(int time, double speed) throws InterruptedException {
        frontLeft.setPower(-speed);
        frontRight.setPower(-speed);
        backLeft.setPower(-speed);
        backRight.setPower(-speed);

        Thread.sleep(time);

        stopAllWheels();

    }

    // TODO: Refactor moveBackward(speed) to RobotClass and update calls in this OpMode
    void moveBackward(double speed) throws InterruptedException {
        frontLeft.setPower(-speed);
        frontRight.setPower(-speed);
        backLeft.setPower(-speed);
        backRight.setPower(-speed);

    }

    // TODO: Refactor moveForward(time, speed) to RobotClass and update calls in this OpMode
    void moveForward(int time, double speed) throws InterruptedException {
        frontLeft.setPower(speed);
        frontRight.setPower(speed);
        backLeft.setPower(speed);
        backRight.setPower(speed);

        Thread.sleep(time);

        stopAllWheels();
    }

    // TODO: Refactor moveForward(sp
    //  eed) to RobotClass and update calls in this OpMode
    void moveForward(double speed) throws InterruptedException {
        frontLeft.setPower(speed);
        frontRight.setPower(speed);
        backLeft.setPower(speed);
        backRight.setPower(speed);

    }

    // TODO: Refactor moveLeft(time, speed) to RobotClass and update calls in this OpMode
    void moveLeft(int time, double speed) throws InterruptedException {
        frontLeft.setPower(-speed);
        frontRight.setPower(speed);
        backLeft.setPower(speed);
        backRight.setPower(-speed);

        Thread.sleep(time);

        stopAllWheels();

    }

    void moveLeft(double speed) {
        frontLeft.setPower(-speed);
        frontRight.setPower(speed);
        backLeft.setPower(speed);
        backRight.setPower(-speed);
    }

    // TODO: Refactor moveRight(time, speed) to RobotClass and update calls in this OpMode
    void moveRight(int time, double speed) throws InterruptedException {
        frontLeft.setPower(speed);
        frontRight.setPower(-speed);
        backLeft.setPower(-speed);
        backRight.setPower(speed);

        Thread.sleep(time);

        stopAllWheels();
    }


    void moveRight(double speed)  {
        frontLeft.setPower(speed);
        frontRight.setPower(-speed);
        backLeft.setPower(-speed);
        backRight.setPower(speed);
    }

    // TODO: Refactor turnRight(time, speed) to RobotClass and update calls in this OpMode
    void turnRight(int time, double speed) throws InterruptedException {
        frontLeft.setPower(speed);
        frontRight.setPower(-speed);
        backLeft.setPower(speed);
        backRight.setPower(-speed);

        Thread.sleep(time);

        stopAllWheels();

    }

    // TODO: Refactor turnLeft(time, speed) to RobotClass and update calls in this OpMode
    void turnLeft(int time, double speed) throws InterruptedException {
        frontLeft.setPower(-speed);
        frontRight.setPower(speed);
        backLeft.setPower(-speed);
        backRight.setPower(speed);

        Thread.sleep(time);

        stopAllWheels();

    }

    // TODO: Refactor stopAllWheels() to RobotClass and update calls in this OpMode
    void stopAllWheels() throws InterruptedException {

        frontLeft.setPower(0);
        frontRight.setPower(0);
        backLeft.setPower(0);
        backRight.setPower(0);

    }

    void stopAllWheelsNoInterrupt()  {

        frontLeft.setPower(0);
        frontRight.setPower(0);
        backLeft.setPower(0);
        backRight.setPower(0);

    }

    void openLeftIntake() {
        intakeLeft.setPosition(0.9);
    }

    void closeLeftIntake() {
        intakeLeft.setPosition(0.3);
    }

    void openRightIntake() {
        intakeRight.setPosition(0);
    }

    void closeRightIntake() {
        intakeRight.setPosition(1.0);
    }

    void lowerRamp() {
        ramp.setPosition(0.58);
        // dont press too hard on mat, otherwise it lifts up the robot and the intake may not work properly
    }

    void liftRamp() {
        ramp.setPosition(0.47);
    }

    void readyIntakePlatform() {
        // get it lower to the ground to get it ready for sweep by intake servo
        indexer.setPosition(0);
    }

    void initIntakePlatform() {
        // keep it slightly up so it doesnt drag on the floor
        indexer.setPosition(0.25);
    }

    void liftIntakePlatform() {
        // rotate it so it gets to the bucket on the linear slide/crane
        indexer.setPosition(0.7);
    }
    void extendCraneVelocity(double vel) {
        crane.setVelocity(vel);
    }
    void extendCraneUseSensorVelocity(double vel) {
        // extend crane till a timeout value or till the sensor detects closeness to backdrop
        final int EXTEND_TIMEOUT = 2000 ; // timeout depends on the speed
        final double BACKDROP_DIST_IN_CM = 8.0 ;
        eTime1.reset();
        crane.setVelocity(vel);
        while((distanceBucket.getDistance(DistanceUnit.CM) > BACKDROP_DIST_IN_CM) && (eTime1.milliseconds() < EXTEND_TIMEOUT)) {

        }
        stopCrane();

    }
    int extendCraneUseColorSensorVelocity(double vel, int timeout_milli, int backdrop_color_val, int slow_time) {
        // extend crane till given timeout value or till the sensor detects proximity to backdrop based on given distance
        // NOTE: timeout depends on the speed
        eTime1.reset();
        crane.setVelocity(vel);
        boolean yPressed = false;
        boolean aPressed = false;
        boolean tPressed = false;
        //while((distanceBucket.getDistance(DistanceUnit.CM) > backdrop_dist_cm) && (crane.getCurrentPosition() < craneMax) && (eTime1.milliseconds() < timeout_milli))
        int colorVal = 0;

        final int BUCKET_COLOR_THRESHOLD = 40 ;

        while((colorVal = colorFieldLine.red()) < BUCKET_COLOR_THRESHOLD) //
        {
            if ((crane.getCurrentPosition() > craneMax) || (eTime1.milliseconds() > timeout_milli))
                break ;

            if((colorVal = colorFieldLine.red()) >= 40) { stopCrane(); break ; }

            if (gamepad2.right_bumper) {
                tPressed = true ;
                break ;
            }
            if((colorVal = colorFieldLine.red()) >= 40) { stopCrane(); break ; }
            if (gamepad1.dpad_right) // note change
                moveLeft(0.3) ;
            else {
                if(gamepad1.dpad_left) // note change
                    moveRight(0.3) ;
                else
                    stopAllWheelsNoInterrupt();
            }
            if((colorVal = colorFieldLine.red()) >= 40) { stopCrane(); break ; }

            if (gamepad2.y) {
                aPressed = false ;
                if (yPressed == false)  {
                    yPressed = true;
                    liftCraneSlightly(0.05);
                    // increase time
                    slow_time = slow_time + 1000;
                }
            }
            if((colorVal = colorFieldLine.red()) >= 40) { stopCrane(); break ; }
            if (gamepad2.a) {
                yPressed = false ;
                if (aPressed == false)  {
                    aPressed = true;
                    liftCraneSlightly(-0.05);
                }
            }
            if((colorVal = colorFieldLine.red()) >= 40) { stopCrane(); break ; }
            if (!(gamepad2.y || gamepad2.a)) {
                yPressed = false ;
                aPressed = false ;
            }

        }
        stopCrane();
        sleep(200);
        stopAllWheelsNoInterrupt();

        telemetry.addData("Color val at stop:", colorVal + "currentVal " + colorFieldLine.red());
        telemetry.update() ;

        if(tPressed == true) {
            return 1;
        }
        //sleep(3000) ;

        // reversing to slow down motor
        crane.setVelocity(-vel*0.15);
        sleep(200) ;
        stopCrane();
        yPressed = false;
        aPressed = false;

        if ((crane.getCurrentPosition() < craneMax) ) {

            crane.setVelocity(vel*0.15);
            int ttime = slow_time ;
            // for the highest angle; increase time
            if (craneAngle.getPosition() > 0.8)
                ttime = ttime + 500;
            eTime1.reset();
            while((eTime1.milliseconds() < ttime) && (colorFieldLine.red() < backdrop_color_val)) {
                if (gamepad2.right_bumper) {
                    tPressed = true ;
                    break ;
                }
                if (gamepad1.dpad_right) // note change
                    moveLeft(0.3) ;
                else {
                    if(gamepad1.dpad_left)
                        moveRight(0.3) ;
                    else
                        stopAllWheelsNoInterrupt();
                }

                /*
                if (gamepad2.y) {
                    aPressed = false ;
                    if (yPressed == false)  {
                        yPressed = true;
                        liftCraneSlightly(0.1);
                    }
                }
                if (gamepad2.a) {
                    yPressed = false ;
                    if (aPressed == false)  {
                        aPressed = true;
                        liftCraneSlightly(-0.1);
                    }
                }
                if (!(gamepad2.y || gamepad2.a)) {
                    yPressed = false ;
                    aPressed = false ;
                }

                 */
            }
            stopCrane();
            stopAllWheelsNoInterrupt();

        }

        if(tPressed == true) {
            return 1;
        }

        return 0 ;



    }
    void retractCrane(double speed) {
        crane.setPower(-1.0*speed);
    }

    void stopCrane() {
        crane.setPower(0.0);
    }

    // see overloaded function ; use appropriately
    void retractCraneHome(double speed) {
        // retract crane till it hits sensor or a certain timeout val
        // NOTE: speed should determine timeout value
        final int RETRACT_TIMEOUT = 2000 ;
        eTime1.reset();
        retractCrane(speed);
        while((!touchCrane.isPressed()) && (eTime1.milliseconds() < RETRACT_TIMEOUT)) {

        }
        stopCrane();
    }

    void retractCraneHome(double speed, int timeout_milli) {
        // retract crane till it hits sensor or given timeout val
        eTime1.reset();
        retractCrane(speed);
        while((!touchCrane.isPressed()) && (eTime1.milliseconds() < timeout_milli)) {

        }
        stopCrane();
    }
    void positionCraneLow() {
        craneAngle.setPosition(0.60);
    }
    void positionCraneMedium() {
        craneAngle.setPosition(0.75);
    }
    void positionCraneHigh() {
        craneAngle.setPosition(0.90);
    }

    void positionCraneBase() {
        craneAngle.setPosition(0);
    }

    void liftCraneSlightly(double incr) {
        craneAngle.setPosition(craneAngle.getPosition() + incr);
    }

    void initDroneLauncher() {
        droneLauncher.setPosition(0);
    }
    void releaseDrone() {
        droneLauncher.setPosition(0.5);
    }
    // using the drone launcher for both drone and suspension hook
    void releaseSuspensionHook() {
        droneLauncher.setPosition(1.0);
    }

    final int PIXEL_DETECTION_THRESHOLD = 110 ;
    // Pixel detection uses alpha; could use others but this seems the most straightforward
    // threshold needs to be calibrated at the match site
    boolean isPixelDetectedLeft() {
        return (pixelDetectorLeft.alpha() > PIXEL_DETECTION_THRESHOLD) ;
    }
    int getPixelDetectionLeftVal() {
        return pixelDetectorLeft.alpha() ;
    }

    boolean isPixelDetectedRight() {
        return (pixelDetectorRight.alpha() > PIXEL_DETECTION_THRESHOLD) ;
    }
    int getPixelDetectionRightVal() {
        return pixelDetectorRight.alpha() ;
    }

    void driveSuspension(double power) {
        suspension.setPower(power);
    }

    void stopSuspension() {
        suspension.setPower(0.0);
    }

    void retractSuspension(double power) {
        suspension.setPower(-power);
    }

    // function for intaking pixel after detection
    // lower ramp, engaging bucket, closing intake servos
    void intakePixel() {
        // sleep values in this function need to be modified
        lowerRamp();
        readyIntakePlatform();
        //sleep(100);
        closeRightIntake();
        closeLeftIntake();
        sleep(1000);
        liftRamp();
        initIntakePlatform();
        openRightIntake();
        openLeftIntake();
        sleep(100);
    }

    void processStickControls() {
        double y = -gamepad1.left_stick_y; // forward or backward
        double x = gamepad1.left_stick_x; // strafing left or right
        double rx = gamepad1.right_stick_x; // turn left (anti-clockwise) or right (clockwise)

        // cube inputs to reduce initial acceleration
        y = Math.pow(y, 3);
        x = Math.pow(x, 3);
        rx = Math.pow(rx, 3);

        double denominator = Math.max(Math.abs(y) + Math.abs(x) + Math.abs(rx), 1);
        frontLeftPower = (y + x + rx) / denominator;
        backLeftPower = (y - x + rx) / denominator;
        frontRightPower = (y - x - rx) / denominator;
        backRightPower = (y + x - rx) / denominator;

        // use the scaling factor
        frontLeftPower *= speedFactor;
        backLeftPower *= speedFactor;
        frontRightPower *= speedFactor;
        backRightPower *= speedFactor;
    }

    void driveMethod() {

        double x = gamepad1.left_stick_x;
        double y = -gamepad1.left_stick_y;
        double turn = gamepad1.right_stick_x;

        double theta = Math.atan2(y, x);
        double power = Math.hypot(x, y);

        double sin = Math.sin(theta - Math.PI/4);
        double cos = Math.cos(theta - Math.PI/4);
        double max = Math.max(Math.abs(sin),Math.abs(cos));

        frontLeftPower   = power * cos/max + turn;
        frontRightPower  = power * sin/max - turn;
        backLeftPower    = power * sin/max + turn;
        backRightPower   = power * cos/max - turn;

        if ((power + Math.abs(turn)) > 1) {

            frontLeftPower   /= power + Math.abs(turn);
            frontRightPower  /= power + Math.abs(turn);
            backLeftPower    /= power + Math.abs(turn);
            backRightPower   /= power + Math.abs(turn);

        }


    }

}
