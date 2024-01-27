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
public class Qualifier2_TeleOp extends LinearOpMode {
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
    public ColorSensor colorFieldLine;

    // Vision portal and vision processing pipelines
    private VisionPortal visionPortal;
    private TfodProcessor tfod;
    private AprilTagProcessor aprilTag;
    boolean targetFound = false;
    boolean streamingEnabled = false ;
    final double TFOD_Y_GAIN = 0.004;    // Forward Speed Control "Gain" for pixel intake.
    final double TFOD_X_GAIN = 0.002;    // Strafe Speed Control "Gain" for pixel intake.
    final double MAX_AUTO_SPEED = 0.8;   //  Clip the approach speed to this max value (adjust for your robot)
    final float CONFIDENCE_THRESHOLD = 0.5f ; // confidence threshold

    final double MAX_AUTO_STRAFE= 0.6;   //  Clip the approach speed to this max value (adjust for your robot)
    final double MAX_AUTO_TURN  = 0.1;   //  Clip the turn speed to this max value (adjust for your robot)

    // helper variables
    ElapsedTime eTime1 = new ElapsedTime() ;
    ElapsedTime eTime2 = new ElapsedTime() ;

    ElapsedTime eTeleOp = new ElapsedTime() ;

    double speedFactor = 1.0 ;

    int craneMax = 4500;

    double cranePosUp = 0.0 ;
    double cranePosDown = 1.0 ;
    double curCranePos = 0.0 ;

    boolean craneUpFlag = false ;
    boolean craneDownFlag = false ;

    public void runOpMode() throws InterruptedException {

        initialize(); // this is only for initializing the drives, servos and sensors
        initTfod();

        initAutonomous(); // initialize servos for autonomous ; raise ramp and open intake
        telemetry.addData("encoder value: ", crane.getCurrentPosition());
        telemetry.addData("Initialized.", "Press START..");
        telemetry.update();

        waitForStart();
        // disable tfod at the start
        visionPortal.setProcessorEnabled(tfod, false);

        cranePosUp = craneAngle.getPosition() ;
        cranePosDown = craneAngle.getPosition() ;

        eTeleOp.reset();

        while (opModeIsActive()) {

            telemetry.addData("crane encoder value: ", crane.getCurrentPosition());
            telemetry.addData("right color sensor value: ", getPixelDetectionRightVal());
            telemetry.addData("left color sensor value: ", getPixelDetectionLeftVal());
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

            // enabled tfod dection is left bumper is pressed
            if ((streamingEnabled == false) && (gamepad1.left_bumper)) {
                // use this to turn on to enable processor
                visionPortal.setProcessorEnabled(tfod, true);
                streamingEnabled = true ;
            }

            // remove tfod processing if not required
            if (gamepad1.right_bumper) {
                visionPortal.setProcessorEnabled(tfod, false);
                streamingEnabled = false ;
            }

            // process the tfod detections if left trigger is pressed and see if any targets are found
            targetFound = false ;
            if ((gamepad1.left_trigger > 0.0) && (streamingEnabled == true)) {
                // then process the detection; if there is a target found, then determine values of drive
                checkForTargetsAndProcessDrive();
            }

            // if targets are found, move robot closer to tfod object based on calculated drive powers
            if (targetFound == true) {
                // NOTE: This runs only when objects are found and the left trigger is pressed
                // provide some indication (light, sound, etc) to show that a target is found
                // NOTE: The drive power values are already set
            }
            else {
                // if trigger is not continuously pressed, then use normal left stick and right stick controls
                // for the drive train
                processStickControls() ;
            }

            // FINALLY run the drive motors
            frontLeft.setPower(frontLeftPower);
            frontRight.setPower(frontRightPower);
            backLeft.setPower(backLeftPower);
            backRight.setPower(backRightPower);

            // individual controls for different sub-systems
            if (gamepad1.a) {
                intakePixel();  // does not throw pixel into bucket
            }

            if (gamepad1.b) {
            }

            if (gamepad1.y) {
                extendCraneUseSensor(0.8, 5000, 15, 2500) ;
                liftCraneSlightly(0.2);
                sleep(50) ;
                retractCraneHome(0.8, 1000);
                positionCraneBase();
                retractCraneHome(0.8, 2000);
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
            if ((cranePower > 0) && (distanceBucket.getDistance(DistanceUnit.CM) < 15))
                cranePower *= 0.2;
            if ((cranePower < 0) && (touchCrane.isPressed()))
                cranePower *= 0.1;
            extendCrane(cranePower);

            // logic for manual lifting of crane
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

            // only for end game
            if(eTeleOp.milliseconds() > 90000) {
                driveSuspension(-gamepad2.right_stick_y);

                if (gamepad2.right_trigger > 0) {
                    releaseDrone();
                }

                if (gamepad2.left_trigger > 0) {
                    releaseSuspensionHook();
                    extendCrane(0.8);
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

    // TODO: Refactor initialize() to RobotClass. OpMode.initialize() should be robot = new RobotClass(); robot.initialize();
    public void initialize() {


        // setting up drive train
        frontRight = hardwareMap.get(DcMotorEx.class, "FrontRight");
        frontLeft = hardwareMap.get(DcMotorEx.class, "FrontLeft");
        backRight = hardwareMap.get(DcMotorEx.class, "BackRight");
        backLeft = hardwareMap.get(DcMotorEx.class, "BackLeft");

        // this ensures that all wheels go forward when applying postive power
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

    void initTfod() {
        final String TFOD_MODEL_ASSET = "pixel_centerstage10820.tflite";
        final String[] LABELS = {
                "green", "purple", "white", "yellow",
        };

        // Create the TensorFlow processor by using a builder.
        tfod = new TfodProcessor.Builder()

                //Use setModelAssetName() if the TF Model is built in as an asset.
                //Use setModelFileName() if you have downloaded a custom team model to the Robot Controller.
                .setModelAssetName(TFOD_MODEL_ASSET)
                //.setModelFileName(TFOD_MODEL_FILE)

                .setModelLabels(LABELS)
                //.setIsModelTensorFlow2(true)
                //.setIsModelQuantized(true)
                //.setModelInputSize(300)
                .setModelAspectRatio(4.0 /3.0)

                .build();

        // Create the vision portal by using a builder.
        VisionPortal.Builder builder = new VisionPortal.Builder();

        builder.setCamera(hardwareMap.get(WebcamName.class, "Webcam 2"));

        // Choose a camera resolution. Not all cameras support all resolutions.
        builder.setCameraResolution(new Size(640, 480));

        //Enable the RC preview (LiveView).  Set "false" to omit camera monitoring.
        builder.enableLiveView(true) ;

        //Set the stream format; MJPEG uses less bandwidth than default YUY2.
        //builder.setStreamFormat(VisionPortal.StreamFormat.YUY2);

        // Choose whether or not LiveView stops if no processors are enabled.
        // If set "true", monitor shows solid orange screen if no processors enabled.
        // If set "false", monitor shows camera view without annotations.
        //builder.setAutoStopLiveView(false);

        // Set and enable the processor.
        builder.addProcessor(tfod);

        // Build the Vision Portal, using the above settings.
        visionPortal = builder.build();

        // Set confidence threshold for TFOD recognitions, at any time.
        tfod.setMinResultConfidence(CONFIDENCE_THRESHOLD);

        //Disable or re-enable the TFOD processor at any time.
        //visionPortal.setProcessorEnabled(tfod, true);

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

    // TODO: Refactor moveRight(time, speed) to RobotClass and update calls in this OpMode
    void moveRight(int time, double speed) throws InterruptedException {
        frontLeft.setPower(speed);
        frontRight.setPower(-speed);
        backLeft.setPower(-speed);
        backRight.setPower(speed);

        Thread.sleep(time);

        stopAllWheels();
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
        ramp.setPosition(0.55);
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

    void extendCrane(double speed) {
        crane.setPower(speed);
    }
    void extendCraneVelocity(double vel) {
        crane.setVelocity(400);
    }
    // see overloaded function ; use appropriately
    void extendCraneUseSensor(double speed) {
        // extend crane till a timeout value or till the sensor detects closeness to backdrop
        final int EXTEND_TIMEOUT = 2000 ; // timeout depends on the speed
        final double BACKDROP_DIST_IN_CM = 8.0 ;
        eTime1.reset();
        crane.setPower(speed);
        while((distanceBucket.getDistance(DistanceUnit.CM) > BACKDROP_DIST_IN_CM) && (eTime1.milliseconds() < EXTEND_TIMEOUT)) {

        }
        stopCrane();
    }
    void extendCraneUseSensor(double speed, int timeout_milli, double backdrop_dist_cm, int slow_time) {
        // extend crane till given timeout value or till the sensor detects proximity to backdrop based on given distance
        // NOTE: timeout depends on the speed
        eTime1.reset();
        crane.setPower(speed);
        while((distanceBucket.getDistance(DistanceUnit.CM) > backdrop_dist_cm) && (crane.getCurrentPosition() < craneMax)) {

        }
        stopCrane();
        //sleep(500);

        if (crane.getCurrentPosition() < 4000) {
            crane.setPower(speed*0.2);
            sleep(slow_time);
            stopCrane();

/*
            crane.setTargetPosition(crane.getCurrentPosition() + (8 * 63)); // calculate 63 encoder ticks per cm
            crane.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
            crane.setPower(speed*0.2);

 */

        }




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
        craneAngle.setPosition(0.55);
    }
    void positionCraneMedium() {
        craneAngle.setPosition(0.70);
    }
    void positionCraneHigh() {
        craneAngle.setPosition(0.85);
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
    void telemetryTfod() {
        List<Recognition> currentRecognitions = tfod.getRecognitions() ;
        telemetry.addData("# Objects detected", currentRecognitions.size()) ;

        for (Recognition recognition : currentRecognitions) {
            telemetry.addData("Pixel ", recognition.getLabel() + " Conf. " + recognition.getConfidence() + " Width " + recognition.getWidth() + " Height " + recognition.getHeight());
        }
    }

    private void initAprilTag() {
        // Create the AprilTag processor by using a builder.
        aprilTag = new AprilTagProcessor.Builder().build();

        // Create the vision portal by using a builder.
        visionPortal = new VisionPortal.Builder()
                .setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"))
                .addProcessor(aprilTag)
                .build();

        setManualExposure(6, 250);  // Use low exposure time to reduce motion blur
    }

    private void    setManualExposure(int exposureMS, int gain) {
        // Wait for the camera to be open, then use the controls

        if (visionPortal == null) {
            return;
        }

        // Make sure camera is streaming before we try to set the exposure controls
        if (visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING) {
            telemetry.addData("Camera", "Waiting");
            telemetry.update();
            while (!isStopRequested() && (visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING)) {
                sleep(20);
            }
            telemetry.addData("Camera", "Ready");
            telemetry.update();
        }

        // Set camera controls unless we are stopping.
        if (!isStopRequested())
        {
            ExposureControl exposureControl = visionPortal.getCameraControl(ExposureControl.class);
            if (exposureControl.getMode() != ExposureControl.Mode.Manual) {
                exposureControl.setMode(ExposureControl.Mode.Manual);
                sleep(50);
            }
            exposureControl.setExposure((long)exposureMS, TimeUnit.MILLISECONDS);
            sleep(20);
            GainControl gainControl = visionPortal.getCameraControl(GainControl.class);
            gainControl.setGain(gain);
            sleep(20);
        }
    }

    public void moveRobot(double x, double y, double yaw) {
        // Calculate wheel powers.
        double leftFrontPower    =  x -y -yaw;
        double rightFrontPower   =  x +y +yaw;
        double leftBackPower     =  x +y -yaw;
        double rightBackPower    =  x -y +yaw;

        // Normalize wheel powers to be less than 1.0
        double max = Math.max(Math.abs(leftFrontPower), Math.abs(rightFrontPower));
        max = Math.max(max, Math.abs(leftBackPower));
        max = Math.max(max, Math.abs(rightBackPower));

        if (max > 1.0) {
            leftFrontPower /= max;
            rightFrontPower /= max;
            leftBackPower /= max;
            rightBackPower /= max;
        }

        frontLeft.setPower(leftFrontPower);
        frontRight.setPower(rightFrontPower);
        backLeft.setPower(leftBackPower);
        backRight.setPower(rightBackPower);
    }

    /*
    void strafeToAprilTag(int tagNumber) throws InterruptedException {

        //tag center - x = 3, y = 16.5
        double tagXPos = 3;
        double tagYPos = 16.5;

        boolean targetFound = false;
        desiredTag  = null;

        List<AprilTagDetection> currentDetections = aprilTag.getDetections();
        for (AprilTagDetection detection : currentDetections) {
            if ((detection.metadata != null) &&
                    (detection.id == tagNumber)  ){
                targetFound = true;
                desiredTag = detection;
                break;  // don't look any further.
            } else {
                telemetry.addData("Unknown Target", "Tag ID %d is not in TagLibrary\n", detection.id);
            }
        }

        if (targetFound) {
            telemetry.addData("Target", "ID %d (%s)", desiredTag.id, desiredTag.metadata.name);
            telemetry.addData("X Value: ",  desiredTag.ftcPose.x);
            telemetry.addData("Y Value: ",  desiredTag.ftcPose.y);


        } else {
            telemetry.addData("Target", "not found") ;
        }

        telemetry.update();


        if (desiredTag != null) {

            double xPos = desiredTag.ftcPose.x;
            double yPos = desiredTag.ftcPose.y;
            double threshold = 0.5;

            if (Math.abs(xPos - tagXPos) > threshold) {

                if (xPos > tagXPos) {

                    moveLeft(25, 0.5);


                } else if (xPos < tagXPos) {

                    moveRight(25, 0.5);

                }
            }
        }

    }
     */

    // function for intaking pixel after detection
    // lower ramp, engaging bucket, closing intake servos
    void intakePixel() {
        // sleep values in this function need to be modified
        lowerRamp();
        readyIntakePlatform();
        sleep(100);
        closeRightIntake();
        closeLeftIntake();
            sleep(1500);
        liftRamp();
        initIntakePlatform();
        openRightIntake();
        openLeftIntake();
        sleep(100);
    }

    void checkForTargetsAndProcessDrive() {
        targetFound = false ;
        if (visionPortal.getProcessorEnabled(tfod) != true)
            return;

        List<Recognition> currentRecognitions = tfod.getRecognitions();
        int numDetections = currentRecognitions.size();
        //telemetry.addData("# Objects Detected", numDetections);
        if (numDetections == 0)
            return ;

        // Step through the list of recognitions and determine which one to align to
        // As a first step assume that you have just one recognition or take the one with the highest confidence
        double curConf = 0.0 ;
        double pixelx=0.0, pixely=0.0, pixelwidth=0.0, pixelheight=0.0 ;
        for (Recognition recognition : currentRecognitions) {
            double x = (recognition.getLeft() + recognition.getRight()) / 2 ;
            double y = (recognition.getTop()  + recognition.getBottom()) / 2 ;

            // there are various ways to process the detections
            // 1. Get the one with the highest confidence
            // 2. Get the left most one
            // 3. Get the one with the largest width
            if (recognition.getConfidence() >= curConf) {
                curConf = recognition.getConfidence() ;
                pixelx = x;
                pixely = y;
                pixelwidth = recognition.getWidth();
                pixelheight = recognition.getHeight();
            }
            //telemetry.addData(""," ");
            //telemetry.addData("Image", "%s (%.0f %% Conf.)", recognition.getLabel(), recognition.getConfidence() * 100);
            //telemetry.addData("- Position", "%.0f / %.0f", x, y);
            //telemetry.addData("- Size", "%.0f x %.0f", recognition.getWidth(), recognition.getHeight());
        }   // end for() loop

        if (curConf < CONFIDENCE_THRESHOLD)
            return ;

        // use this to get your xdrive and ydrive values
        double xError, yError ;
        double drive, strafe ; // no yaw
        xError = 280-pixelx;
        yError = 330-pixely;

        // NOTE: The IF statement below is CRITICAL
        if (yError < 30) { // 30 (pixels) is used as threshold here, but it can be changed to a different value
            targetFound = false;
            return ;
        }

        targetFound = true ;

        drive  = Range.clip(yError * TFOD_Y_GAIN, -MAX_AUTO_SPEED, MAX_AUTO_SPEED);
        strafe = Range.clip(xError * TFOD_X_GAIN, -MAX_AUTO_STRAFE, MAX_AUTO_STRAFE);

        // use the classic formula to get the drive values
        // note that there is no yaw/turn to put in this formula
        frontLeftPower = drive - strafe;
        backLeftPower = drive + strafe;
        frontRightPower = drive + strafe;
        backRightPower = drive - strafe ;

        // Normalize wheel powers to be less than 1.0
        double max = Math.max(Math.abs(frontLeftPower), Math.abs(frontRightPower));
        max = Math.max(max, Math.abs(backLeftPower));
        max = Math.max(max, Math.abs(backRightPower));

        if (max > 1.0) {
            frontLeftPower /= max;
            frontRightPower /= max;
            backLeftPower /= max;
            backRightPower /= max;
        }

    }
    void moveToPixelUseCamera()  {

        double drive = 0;
        double strafe = 0;
        double xError = 0.0, yError = 0.0;

        targetFound = false ; // NOTE: comment this out to bypass pixel detection
        while (eTime1.milliseconds() < 5000) { // assuming here that it wont take too much time to get to target
            //while (opModeIsActive()) {
            if (targetFound == true)
                break;
            //strafeToAprilTag(3);
            //telemetryTfod();
            //telemetry.update() ;

            List<Recognition> currentRecognitions = tfod.getRecognitions();
            int numDetections = currentRecognitions.size();
            telemetry.addData("# Objects Detected", numDetections);

            // Step through the list of recognitions and determine which one to align to
            // As a first step assume that you have just one recognition or take the one with the highest confidence
            double curConf = 0.0 ;
            double pixelx=0.0, pixely=0.0, pixelwidth=0.0, pixelheight=0.0 ;
            for (Recognition recognition : currentRecognitions) {
                double x = (recognition.getLeft() + recognition.getRight()) / 2 ;
                double y = (recognition.getTop()  + recognition.getBottom()) / 2 ;

                if (recognition.getConfidence() >= curConf) {
                    curConf = recognition.getConfidence() ;
                    pixelx = x;
                    pixely = y;
                    pixelwidth = recognition.getWidth();
                    pixelheight = recognition.getHeight();
                }
                telemetry.addData(""," ");
                telemetry.addData("Image", "%s (%.0f %% Conf.)", recognition.getLabel(), recognition.getConfidence() * 100);
                telemetry.addData("- Position", "%.0f / %.0f", x, y);
                telemetry.addData("- Size", "%.0f x %.0f", recognition.getWidth(), recognition.getHeight());
            }   // end for() loop

            if (curConf != 0.0) {
                xError = 280-pixelx;
                yError = 330-pixely;

                if (yError < 30) { // 30 (pixels) is used as threshold here, but it can be changed to a different value
                    targetFound = true;
                }

                drive  = Range.clip(yError * TFOD_Y_GAIN, -MAX_AUTO_SPEED, MAX_AUTO_SPEED);
                strafe = Range.clip(xError * TFOD_X_GAIN, -MAX_AUTO_STRAFE, MAX_AUTO_STRAFE);

                // there are times when there is no pixels detected
                // In that case, keep drive train enabled at low power for a small amount of time only
                // there needs to be some extra logic over here for that
                //
            } else {
                drive = 0.05 ;
                strafe = 0 ;

                // for using this in teleop, replace above with gamepad input
                // see example RobotAutoDriveToAprilTagOmni.java example
                /*
                if (eTime2.milliseconds() < 2000) {
                    break ;
                }
                 */
            }

            telemetry.addData("- Error", "%.0f / %.0f", xError, yError);
            telemetry.addData("- Power", "D: " + drive + " S: " + strafe);

            moveRobot(drive,strafe,0);
            telemetry.update();
            sleep(20) ;
        }

    }

}
