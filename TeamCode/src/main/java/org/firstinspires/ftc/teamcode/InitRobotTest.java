package org.firstinspires.ftc.teamcode;


import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.BuiltinCameraDirection;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.tfod.TfodProcessor;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;


@Autonomous
public class InitRobotTest extends LinearOpMode {

    // drive train motors
    public DcMotorEx frontLeft;
    public DcMotorEx frontRight;
    public DcMotorEx backLeft;
    public DcMotorEx backRight;

    // crane linear slide and lifter
    public DcMotorEx crane;
    public Servo craneAngle;

    // the rotating platform for intake
    public Servo indexer;

    // The servo used for intake (left and right - with the robot's perspective)
    public Servo intakeLeft;
    public Servo intakeRight;

    // the short ramp used for the intake mechanism
    public Servo ramp;

    // the drone launcher
    public Servo droneLauncher;

    // sensors used for pixel intake
    private ColorSensor pixelDetectorRight;
    private ColorSensor pixelDetectorLeft;

    // distance sensor on the bucket to detect when bucket is close to backdrop
    private DistanceSensor distanceBucket;

    // touch sensor used with linear slide
    public TouchSensor touchCrane;
    // Color sensor at the back of the robot used for detecting lines on the field
    public ColorSensor colorFieldLine;

    // Vision portal and vision processing pipeline
    private VisionPortal visionPortal;
    private TfodProcessor tfod;

    // helper variables
    ElapsedTime eTime1 = new ElapsedTime() ;
    ElapsedTime eTime2 = new ElapsedTime() ;


    @Override
    public void runOpMode() throws InterruptedException {

        initialize();

        initializeVision() ;

        waitForStart();

        if(opModeIsActive()) {
            eTime1.reset();

            while (eTime1.milliseconds() < 20000) {
                telemetryTfod();
                telemetry.update();
                sleep(100);
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
        crane.setDirection(DcMotorEx.Direction.REVERSE);
        crane.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        //servos
        indexer = hardwareMap.get(Servo.class, "Indexer");
        intakeLeft = hardwareMap.get(Servo.class, "IntakeLeft");
        intakeRight = hardwareMap.get(Servo.class, "IntakeRight");
        craneAngle = hardwareMap.get(Servo.class, "CraneAngle");
        ramp = hardwareMap.get(Servo.class, "Ramp");
        droneLauncher = hardwareMap.get(Servo.class, "DroneLauncher");


        /*
        ramp.setPosition(0.3);
        indexer.setPosition(0.25);
        intakeLeft.setPosition(0.1);
        intakeRight.setPosition(0.3);
         */


        //sensors
        touchCrane = hardwareMap.get(TouchSensor.class, "Touch");
        colorFieldLine = hardwareMap.get(ColorSensor.class, "Color");

        pixelDetectorLeft = hardwareMap.get(ColorSensor.class, "PixelDetectorLeft");
        pixelDetectorRight = hardwareMap.get(ColorSensor.class, "PixelDetectorRight");

        distanceBucket = hardwareMap.get(DistanceSensor.class, "DistanceBucket") ;
    }

    void initializeVision() {
        final String TFOD_MODEL_ASSET = "CenterStage.tflite";
        final String[] LABELS = {
                "Pixel",
        };

        // Create the TensorFlow processor by using a builder.
        tfod = new TfodProcessor.Builder()

                // Use setModelAssetName() if the TF Model is built in as an asset.
                // Use setModelFileName() if you have downloaded a custom team model to the Robot Controller.
                .setModelAssetName(TFOD_MODEL_ASSET)
                //.setModelFileName(TFOD_MODEL_FILE)

                .setModelLabels(LABELS)
                //.setIsModelTensorFlow2(true)
                //.setIsModelQuantized(true)
                //.setModelInputSize(300)
                //.setModelAspectRatio(16.0 / 9.0)

                .build();

        // Create the vision portal by using a builder.
        VisionPortal.Builder builder = new VisionPortal.Builder();

        builder.setCamera(hardwareMap.get(WebcamName.class, "Webcam 2"));

        // Choose a camera resolution. Not all cameras support all resolutions.
        //builder.setCameraResolution(new Size(640, 480));

        // Enable the RC preview (LiveView).  Set "false" to omit camera monitoring.
        //builder.enableCameraMonitoring(true);

        // Set the stream format; MJPEG uses less bandwidth than default YUY2.
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
        //tfod.setMinResultConfidence(0.75f);

        // Disable or re-enable the TFOD processor at any time.
        //visionPortal.setProcessorEnabled(tfod, true);

    }


    // TODO: Refactor moveBackward(time, speed) to RobotClass and update calls in this OpMode
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

    // TODO: Refactor moveBackward(speed) to RobotClass and update calls in this OpMode
    void moveBackward(double speed) throws InterruptedException {
        frontLeft.setPower(speed);
        frontRight.setPower(speed);
        backLeft.setPower(speed);
        backRight.setPower(speed);

    }

    // TODO: Refactor moveForward(time, speed) to RobotClass and update calls in this OpMode
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

    // TODO: Refactor moveForward(speed) to RobotClass and update calls in this OpMode
    void moveForward(double speed) throws InterruptedException {
        frontLeft.setPower(-speed);
        frontRight.setPower(-speed);
        backLeft.setPower(-speed);
        backRight.setPower(-speed);

    }

    // TODO: Refactor moveLeft(time, speed) to RobotClass and update calls in this OpMode
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

    // TODO: Refactor moveRight(time, speed) to RobotClass and update calls in this OpMode
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


    // TODO: Refactor turnRight(time, speed) to RobotClass and update calls in this OpMode
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

    // TODO: Refactor turnLeft(time, speed) to RobotClass and update calls in this OpMode
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

    // TODO: Refactor stopAllWheels() to RobotClass and update calls in this OpMode
    void stopAllWheels() throws InterruptedException {

        frontLeft.setPower(0);
        frontRight.setPower(0);
        backLeft.setPower(0);
        backRight.setPower(0);

    }

    void openLeftIntake() {
        intakeLeft.setPosition(1.0);
    }

    void closeLeftIntake() {
        intakeLeft.setPosition(0.2);
    }

    void openRightIntake() {
        intakeRight.setPosition(0.32);
    }

    void closeRightIntake() {
        intakeRight.setPosition(0.8);
    }

    void lowerRamp() {
        ramp.setPosition(0.4);
        // dont press too hard on mat, otherwise it lifts up the robot and the intake may not work properly
    }

    void liftRamp() {
        ramp.setPosition(0.3);
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
        indexer.setPosition(1.0);
    }

    void extendCrane(double speed) {
        crane.setPower(speed);
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
    void extendCraneUseSensor(double speed, int timeout_milli, double backdrop_dist_cm) {
        // extend crane till given timeout value or till the sensor detects proximity to backdrop based on given distance
        // NOTE: timeout depends on the speed
        eTime1.reset();
        crane.setPower(speed);
        while((distanceBucket.getDistance(DistanceUnit.CM) > backdrop_dist_cm) && (eTime1.milliseconds() < timeout_milli)) {

        }
        stopCrane();
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
        craneAngle.setPosition(0.25);
    }
    void positionCraneMedium() {
        craneAngle.setPosition(0.5);
    }
    void positionCraneHigh() {
        craneAngle.setPosition(1.0);
    }

    void positionCraneBase() {
        craneAngle.setPosition(0);
    }

    void initDroneLauncher() {
        droneLauncher.setPosition(0);
    }
    void releaseDrone() {
        droneLauncher.setPosition(1);
    }

    final int PIXEL_DETECTION_THRESHOLD = 100 ;
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

    void telemetryTfod() {
        List<Recognition> currentRecognitions = tfod.getRecognitions() ;
        telemetry.addData("# Objects detected", currentRecognitions.size()) ;

        for (Recognition recognition : currentRecognitions) {
            telemetry.addData("Pixel ", recognition.getLabel() + " Conf. " + recognition.getConfidence() + " Width " + recognition.getWidth() + " Height " + recognition.getHeight());
        }
    }
}
