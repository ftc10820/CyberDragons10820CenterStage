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
    // Color sensor on bucket
    public ColorSensor colorBucket;


    // helper variables
    ElapsedTime eTime1 = new ElapsedTime() ;
    ElapsedTime eTime2 = new ElapsedTime() ;

    ElapsedTime eTeleOp = new ElapsedTime() ;

    double speedFactor = 0.75;

    // crane specific vars
    final int CRANE_MAX_ENCODER_VAL = 4500;
    final double CRANE_MAX_VELOCITY = 4000 ;

    final double CRANE_ANGLE_HOME = 0.0 ;
    final double CRANE_ANGLE_HOME_LOW = 0.30 ;
    final double CRANE_ANGLE_LOW = 0.65 ;
    final double CRANE_ANGLE_LOW_MEDIUM = 0.7 ;
    final double CRANE_ANGLE_MEDIUM = 0.8 ;
    final double CRANE_ANGLE_MEDIUM_HIGH = 0.85 ;
    final double CRANE_ANGLE_HIGH = 0.95 ;

    final int POS1 = 1 ;
    final int POS2 = 2;
    final int POS3 = 3;
    final int POS0 = 0 ;

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
            telemetry.addData("bucket: ", "alpha " + colorBucket.alpha() + " red " + colorBucket.red() + " green " + colorBucket.green() + " blue " + colorBucket.blue() + " hue " + colorBucket.argb());
            telemetry.update();

            //drivetrain
            driveMethod();

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
                int colorVal = 700 ; // default value
                int curPos = POS0 ;
                double newPos = CRANE_ANGLE_LOW ;
                double craneAngle = getCurrentCranePos() ;
                if ((craneAngle > CRANE_ANGLE_HOME_LOW) && (craneAngle < CRANE_ANGLE_LOW_MEDIUM)) {
                    colorVal = 600 ;
                    curPos = POS1 ;
                    newPos = CRANE_ANGLE_LOW + 0.1 ;
                }
                if ((craneAngle > CRANE_ANGLE_LOW_MEDIUM) && (craneAngle < CRANE_ANGLE_MEDIUM_HIGH)) {
                    colorVal = 700 ;
                    curPos = POS2 ;
                    newPos = CRANE_ANGLE_MEDIUM + 0.1 ;
                }
                if (craneAngle >= CRANE_ANGLE_HIGH) {
                    colorVal = 150 ;
                    curPos = POS3 ;
                    newPos = CRANE_ANGLE_HIGH + 0.1 ;
                }

                int retval = extendCraneUseColorSensorVelocity(2500, 5000, colorVal, 5000);
                //sleep(200) ;
                // the below part cane be made asynchronous
                if (retval == 0) {
                    setCranePos(newPos);
                    sleep(1000) ;
                    retractCraneHomeVelocity(CRANE_MAX_VELOCITY, 2000);
                    positionCraneBase();
                    retractCraneHomeVelocity(CRANE_MAX_VELOCITY, 2000);
                }
            }

            if (gamepad1.x) {
                retractCraneHomeVelocity(CRANE_MAX_VELOCITY, 5000);
            }

            if (gamepad2.y) {
                positionCraneMedium();
            } else if (gamepad2.a) {
                positionCraneBase();
            } else if (gamepad2.x) {
                positionCraneLow();
            } else if (gamepad2.b)
                positionCraneHigh();

            if (gamepad1.dpad_right) // note change
                moveLeft(0.3);

            if(gamepad1.dpad_left) // note change
                moveRight(0.3);

            // take distance sensor and touch sensor into account
            // logic for manual control of linear slide
            double cranePower = -gamepad2.left_stick_y;
            /*if ((cranePower > 0) && (distanceBucket.getDistance(DistanceUnit.CM) < 15))
                cranePower *= 0.2;
            if ((cranePower < 0) && (touchCrane.isPressed()))
                cranePower *= 0.1;*/
            extendCraneVelocity(cranePower*CRANE_MAX_VELOCITY);

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
            if(eTeleOp.milliseconds() > 90000) {
            driveSuspension(-gamepad2.right_stick_y);

            if (gamepad2.right_trigger > 0) {
                releaseDrone();
            }

            if (gamepad2.left_trigger > 0) {
                releaseSuspensionHook();
                extendCraneVelocity(CRANE_MAX_VELOCITY);
                sleep(100);
                stopCrane();
            }
            }

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
        colorBucket = hardwareMap.get(ColorSensor.class, "Color");

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
        indexer.setPosition(0.3);
    }

    void liftIntakePlatform() {
        // rotate it so it gets to the bucket on the linear slide/crane
        indexer.setPosition(0.7);
    }
    void extendCraneVelocity(double vel) {
        crane.setVelocity(vel);
    }
    int extendCraneUseDistanceSensorVelocity(double vel, int timeout_milli, int backdrop_dist_cm, int slow_time) {
        // extend crane till a timeout value or till the sensor detects closeness to backdrop
        // simpler function than when color sensor is used
        eTime1.reset();
        crane.setVelocity(vel);
        boolean yPressed = false;
        boolean aPressed = false;
        boolean tPressed = false;
        double distval = 0.0 ;

        while((distval = distanceBucket.getDistance(DistanceUnit.CM)) > backdrop_dist_cm) //
        {
            if ((crane.getCurrentPosition() > CRANE_MAX_ENCODER_VAL) || (eTime1.milliseconds() > timeout_milli))
                break ;

            if((distval = distanceBucket.getDistance(DistanceUnit.CM)) <= backdrop_dist_cm) { stopCrane(); break ; }

            if (gamepad2.right_bumper) {
                tPressed = true ;
                break ;
            }
            if((distval = distanceBucket.getDistance(DistanceUnit.CM)) <= backdrop_dist_cm) { stopCrane(); break ; }
            if (gamepad1.dpad_right) // note change
                moveLeft(0.3) ;
            else {
                if(gamepad1.dpad_left) // note change
                    moveRight(0.3) ;
                else
                    stopAllWheelsNoInterrupt();
            }
            if((distval = distanceBucket.getDistance(DistanceUnit.CM)) <= backdrop_dist_cm) { stopCrane(); break ; }

            if (gamepad2.y) {
                aPressed = false ;
                if (yPressed == false)  {
                    yPressed = true;
                    liftCraneSlightly(0.05);
                    // increase time
                    slow_time = slow_time + 1000;
                }
            }
            if((distval = distanceBucket.getDistance(DistanceUnit.CM)) <= backdrop_dist_cm) { stopCrane(); break ; }
            if (gamepad2.a) {
                yPressed = false ;
                if (aPressed == false)  {
                    aPressed = true;
                    liftCraneSlightly(-0.05);
                }
            }
            if((distval = distanceBucket.getDistance(DistanceUnit.CM)) <= backdrop_dist_cm) { stopCrane(); break ; }
            if (!(gamepad2.y || gamepad2.a)) {
                yPressed = false ;
                aPressed = false ;
            }

        }
        stopCrane();
        sleep(200);
        stopAllWheelsNoInterrupt();

        telemetry.addData("Distance val at stop:", distval );
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

        if ((crane.getCurrentPosition() < CRANE_MAX_ENCODER_VAL) ) {

            crane.setVelocity(vel*0.15);
            int ttime = slow_time ;
            // for the highest angle; increase time
            if (craneAngle.getPosition() > 0.8)
                ttime = ttime + 500;
            eTime1.reset();
            while(eTime1.milliseconds() < ttime) {
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
    int extendCraneUseColorSensorVelocity(double vel, int timeout_milli, int backdrop_color_val, int slow_time) {
        // extend crane till given timeout value or till the sensor detects proximity to backdrop based on color sensor
        boolean yPressed = false;
        boolean aPressed = false;
        boolean tPressed = false;
        int colorVal = 0;

        final int BUCKET_COLOR_FINAL_THRESHOLD = 40 ;
        final int BUCKET_COLOR_FIRST_THRESHOLD = 35 ;

        eTime1.reset();
        crane.setVelocity(vel);
        // there is a way to do proportional velocity reduction rather than keep velocity constant
        // FOR FUTURE WORK
        while((colorVal = colorBucket.red()) < BUCKET_COLOR_FINAL_THRESHOLD) //
        {
            // see if this is good
            /*
            if (colorVal > BUCKET_COLOR_FIRST_THRESHOLD) {
                crane.setVelocity(vel*0.5);
            }
             */

            if ((crane.getCurrentPosition() > CRANE_MAX_ENCODER_VAL) || (eTime1.milliseconds() > timeout_milli))
                break ;

            if((colorVal = colorBucket.red()) >= BUCKET_COLOR_FINAL_THRESHOLD) { stopCrane(); break ; }

            if (gamepad2.right_bumper) {
                tPressed = true ;
                break ;
            }
            if((colorVal = colorBucket.red()) >= BUCKET_COLOR_FINAL_THRESHOLD) { stopCrane(); break ; }
            if (gamepad1.dpad_right) // note change
                moveLeft(0.3) ;
            else {
                if(gamepad1.dpad_left) // note change
                    moveRight(0.3) ;
                else
                    stopAllWheelsNoInterrupt();
            }
            if((colorVal = colorBucket.red()) >= BUCKET_COLOR_FINAL_THRESHOLD) { stopCrane(); break ; }

            if (gamepad2.y) {
                aPressed = false ;
                if (yPressed == false)  {
                    yPressed = true;
                    liftCraneSlightly(0.05);
                    // increase time
                    slow_time = slow_time + 1000;
                }
            }
            if((colorVal = colorBucket.red()) >= BUCKET_COLOR_FINAL_THRESHOLD) { stopCrane(); break ; }
            if (gamepad2.a) {
                yPressed = false ;
                if (aPressed == false)  {
                    aPressed = true;
                    liftCraneSlightly(-0.05);
                }
            }
            if((colorVal = colorBucket.red()) >= BUCKET_COLOR_FINAL_THRESHOLD) { stopCrane(); break ; }
            if (!(gamepad2.y || gamepad2.a)) {
                yPressed = false ;
                aPressed = false ;
            }

        }
        stopCrane();
        sleep(200);
        stopAllWheelsNoInterrupt();

        telemetry.addData("Color val at stop:", colorVal + "currentVal " + colorBucket.red());
        telemetry.update() ;

        if(tPressed == true) {
            return 1;
        }
        //sleep(3000) ;

        // reversing to slow down motor
        // check if this is useful
        retractCraneVelocity(vel*0.15);
        sleep(200) ;
        stopCrane();

        yPressed = false;
        aPressed = false;

        if ((crane.getCurrentPosition() < CRANE_MAX_ENCODER_VAL) ) {

            crane.setVelocity(vel*0.15);
            int ttime = slow_time ;
            // for the highest angle; increase time
            if (craneAngle.getPosition() > 0.8)
                ttime = ttime + 500;
            eTime1.reset();
            while(colorBucket.red() < backdrop_color_val) {
                if ((crane.getCurrentPosition() > CRANE_MAX_ENCODER_VAL) || (eTime1.milliseconds() > ttime))
                    break ;
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
    void retractCraneVelocity(double vel) {
        crane.setVelocity(-1.0*vel);
    }

    void stopCrane() {
        crane.setVelocity(0.0);
    }
    void stopCraneVelocity() {
        crane.setVelocity(0.0);
    }

    void retractCraneHomeVelocity(double vel, int timeout_milli) {
        // retract crane till it hits sensor or given timeout val
        eTime1.reset();
        retractCraneVelocity(vel);
        while((!touchCrane.isPressed()) && (eTime1.milliseconds() < timeout_milli)) {

        }
        stopCrane();
    }
    void positionCraneLow() {
        craneAngle.setPosition(CRANE_ANGLE_LOW);
    }
    void positionCraneMedium() {
        craneAngle.setPosition(CRANE_ANGLE_MEDIUM);
    }
    void positionCraneHigh() {
        craneAngle.setPosition(CRANE_ANGLE_HIGH);
    }

    void positionCraneBase() {
        craneAngle.setPosition(CRANE_ANGLE_HOME);
    }

    double getCurrentCranePos() {
        return craneAngle.getPosition();
    }

    void setCranePos(double newPos) {
        if (newPos > 1.0)
            newPos = 1.0;
        if (newPos < 0.0)
            newPos = 0.0 ;
        craneAngle.setPosition(newPos);
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
        sleep(5 00);
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
