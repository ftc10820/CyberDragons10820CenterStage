package org.firstinspires.ftc.teamcode;

import android.util.Size;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.acmerobotics.roadrunner.trajectory.constraints.AngularVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.MinVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TrajectoryVelocityConstraint;
import com.acmerobotics.roadrunner.trajectory.constraints.TranslationalVelocityConstraint;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.ExposureControl;
import org.firstinspires.ftc.robotcore.external.hardware.camera.controls.GainControl;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.vision.tfod.TfodProcessor;

import java.util.Arrays;
import java.util.List;
import java.util.concurrent.TimeUnit;


@Config
@Autonomous
@Disabled
public class AprilTagAuto extends LinearOpMode {

    //private org.firstinspires.ftc.vision.apriltag.AprilTagDetection desiredTag = null;

    public SampleMecanumDrive drive;

    // crane linear slide and lifter
    public DcMotor crane;
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
    private TfodProcessor tfod;
    private AprilTagProcessor aprilTag;

    private VisionPortal visionPortal;

    //april tag ftc dash variables

     int zone = 1;

     public static int aprilTagZone = 1;

    public static double currentBearing = 10.0;
    public static double currentRange = 10.0;

    public static double currentYaw = 10.0;

    public static double desiredYaw = 10.0;

    public static double strafeCorrection = 10.0;

    public static double strafeDistance = 5.0;
    public static double turnDistance = 0;



    VisionSubsystem vision;


    // helper variables
    ElapsedTime eTime1 = new ElapsedTime() ;
    ElapsedTime eTime2 = new ElapsedTime() ;

    // variables specific to AprilTags
    final double DESIRED_DISTANCE = 12.0; //  this is how close the camera should get to the target (inches)

    //  Set the GAIN constants to control the relationship between the measured position error, and how much power is
    //  applied to the drive motors to correct the error.
    //  Drive = Error * Gain    Make these values smaller for smoother control, or larger for a more aggressive response.
    final double SPEED_GAIN  =  0.1  ;   //  Forward Speed Control "Gain". eg: Ramp up to 50% power at a 25 inch error.   (0.50 / 25.0)
    final double STRAFE_GAIN =  0.015 ;   //  Strafe Speed Control "Gain".  eg: Ramp up to 25% power at a 25 degree Yaw error.   (0.25 / 25.0)
    final double TURN_GAIN   =  0.01  ;   //  Turn Control "Gain".  eg: Ramp up to 25% power at a 25 degree error. (0.25 / 25.0)
    final double TFOD_Y_GAIN = 0.004;    // Forward Speed Control "Gain" for pixel intake.
    final double TFOD_X_GAIN = 0.002;    // Strafe Speed Control "Gain" for pixel intake.
    final double MAX_AUTO_SPEED = 0.8;   //  Clip the approach speed to this max value (adjust for your robot)

    final double MAX_AUTO_STRAFE= 0.6;   //  Clip the approach speed to this max value (adjust for your robot)
    final double MAX_AUTO_TURN  = 0.1;   //  Clip the turn speed to this max value (adjust for your robot)


    private static final int DESIRED_TAG_ID = 2;     // Choose the tag you want to approach or set to -1 for ANY tag.
    public static AprilTagDetection desiredTag = null;     // Used to hold the data for a detected AprilTag

    public Trajectory strafeLeftAprilTag;

    public Trajectory strafeRightAprilTag;


    @Override
    public void runOpMode() throws InterruptedException {

        initialize();
        drive = new SampleMecanumDrive(hardwareMap);

        Pose2d startPose = new Pose2d(0, 0, Math.toRadians(180));
        drive.setPoseEstimate(startPose);//april tag paths


        initAprilTag();
        telemetryAprilTag();
        telemetry.update();

        waitForStart();
        if (opModeIsActive()) {

            if (aprilTagAdjustment() == -1) {

                drive.turn(turnDistance);

                strafeRightAprilTag = drive.trajectoryBuilder(startPose.plus(new Pose2d(0,0, Math.toRadians(turnDistance))))
                        .strafeRight(Math.abs(strafeDistance))
                        .build();

                drive.followTrajectory(strafeRightAprilTag);

            } else if (aprilTagAdjustment() == 1) {

                drive.turn(turnDistance);

                strafeLeftAprilTag = drive.trajectoryBuilder(startPose.plus(new Pose2d(0,0, Math.toRadians(turnDistance))))
                        .strafeLeft(Math.abs(strafeDistance))
                        .build();

                drive.followTrajectory(strafeLeftAprilTag);

            }

        }

    }

    public void initialize() {

        //crane motor
        crane = hardwareMap.get(DcMotorEx.class, "Crane");
        crane.setDirection(DcMotorEx.Direction.REVERSE);
        crane.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

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


        //sensors
        touchCrane = hardwareMap.get(TouchSensor.class, "Touch");
        colorFieldLine = hardwareMap.get(ColorSensor.class, "Color");

        pixelDetectorLeft = hardwareMap.get(ColorSensor.class, "PixelDetectorLeft");
        pixelDetectorRight = hardwareMap.get(ColorSensor.class, "PixelDetectorRight");

        distanceBucket = hardwareMap.get(DistanceSensor.class, "DistanceBucket");




    }

    void openLeftIntake() {
        intakeLeft.setPosition(1.0);
    }

    void closeLeftIntake() {
        intakeLeft.setPosition(0.3);
    }

    void openRightIntake() {
        intakeRight.setPosition(0.32);
    }

    void closeRightIntake() {
        intakeRight.setPosition(0.6);
    }

    void initAutonomous() {
        liftRamp();
        initIntakePlatform();
        openRightIntake();
        openLeftIntake();
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
        tfod.setMinResultConfidence(0.5f);

        //Disable or re-enable the TFOD processor at any time.
        //visionPortal.setProcessorEnabled(tfod, true);

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
    void extendCraneUseSensor(double speed, int timeout_milli, double backdrop_dist_cm, int slow_time) {
        // extend crane till given timeout value or till the sensor detects proximity to backdrop based on given distance
        // NOTE: timeout depends on the speed
        eTime1.reset();
        crane.setPower(speed);
        while((distanceBucket.getDistance(DistanceUnit.CM) > backdrop_dist_cm) && (eTime1.milliseconds() < timeout_milli)) {

        }
        stopCrane();
        crane.setPower(0.2);
        sleep(slow_time) ;
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
        craneAngle.setPosition(0.53);
    }
    void positionCraneMedium() {
        craneAngle.setPosition(0.65);
    }
    void positionCraneHigh() {
        craneAngle.setPosition(0.85);
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

    // function for intaking pixel after detection
    // lower ramp, engaging bucket, closing intake servos
    void intakePixel() {
        // sleep values in this function need to be modified
        lowerRamp();
        readyIntakePlatform();
        sleep (1000) ;
        closeRightIntake();
        closeLeftIntake();
        sleep(3000) ;
        openRightIntake();
        openLeftIntake();

        liftRamp();
        initIntakePlatform();
        sleep(2000) ;

    }
    private double getDistanceToAprilTag(int zone){
        List<AprilTagDetection> currentDetections = aprilTag.getDetections();
        for (AprilTagDetection detection : currentDetections) {
            telemetry.addData("April Tag " + detection.id + " ", detection.ftcPose.x);
            telemetry.addData("April Tag " + detection.id + " ", detection.ftcPose.y);
            if(detection.id == zone){
                    return detection.ftcPose.x;
            }
        }
        telemetry.update();
        return 0.0;
    }
    boolean strafeToAprilTag(int tagNumber) throws InterruptedException {


        //tag center - x = 3, y = 16.5
        double tagXPos = 3;
        double tagYPos = 16.5;

        boolean targetFound = false;
        boolean xAligned = false;
        desiredTag = null;

        List<AprilTagDetection> currentDetections = aprilTag.getDetections();
        for (AprilTagDetection detection : currentDetections) {
            if ((detection.metadata != null) &&
                    (detection.id == tagNumber)) {
                targetFound = true;
                desiredTag = detection;
                break;  // don't look any further.
            } else {
                telemetry.addData("Unknown Target", "Tag ID %d is not in TagLibrary\n", detection.id);
            }
        }

        if (targetFound) {
            telemetry.addData("Target", "ID %d (%s)", desiredTag.id, desiredTag.metadata.name);
            telemetry.addData("X Value: ", desiredTag.ftcPose.x);
            telemetry.addData("Y Value: ", desiredTag.ftcPose.y);


        } else {
            telemetry.addData("Target", "not found");
        }

        telemetry.update();


        if (desiredTag != null) {

            double xPos = desiredTag.ftcPose.x;
            double yPos = desiredTag.ftcPose.y;
            double threshold = 0.5;

            if (Math.abs(xPos - tagXPos) > threshold) {

                if (xPos > tagXPos) {

                    drive.moveLeft(25, 0.5);


                } else if (xPos < tagXPos) {

                    drive.moveRight(25, 0.5);

                }
            }else{
                xAligned = true;
            }


            if (Math.abs(yPos - tagYPos) > threshold) {

                if (yPos > tagYPos) {

                    drive.moveBackwards(25, 0.1);


                } else if (yPos < tagYPos) {

                    drive.moveForward(25, 0.1);

                }
            }else{
                if(xAligned){
                    return true;
                }
            }

        }
        return false;
    }
    private boolean rightAprilTag(int zone){
        List<AprilTagDetection> currentDetections = aprilTag.getDetections();
        for (AprilTagDetection detection : currentDetections) {
            if(detection.id == zone){
                return true;
            }
        }
        return false;
    }


    int aprilTagAdjustment() {

        int direction = 0;

        List<AprilTagDetection> currentDetections = aprilTag.getDetections();
        for (AprilTagDetection detection : currentDetections) {
            if ((detection.metadata != null) && (detection.id == aprilTagZone)) {
                desiredTag = detection;
                break;  // don't look any further.
            } else {

                desiredTag = detection;

            }
        }

        if (desiredTag == null) {

            return 0;
        }

        currentBearing = desiredTag.ftcPose.bearing;
        currentRange = desiredTag.ftcPose.range;
        currentYaw = desiredTag.ftcPose.yaw;

        telemetry.addData("tag using: ", desiredTag.id);
        telemetry.addData("updated bearing: ", currentBearing);
        telemetry.addData("updated range: ", currentRange);
        telemetry.addData("updated yaw: ", currentYaw);
        telemetry.update();


        //TODO: i think the strafe correction should be added after the trig calculation
        //TODO: you may want to add telemetry here too to see what value are going into the calculation and the result
        strafeDistance = currentRange * (Math.sin((Math.toRadians(currentBearing + strafeCorrection))));
        turnDistance = Math.toRadians(-1 * (desiredYaw - currentYaw));



        // correction if detected tag isnt desired zone
        strafeDistance += 8.0 * (aprilTagZone - desiredTag.id);

        telemetry.addData("strafeDistance: ", strafeDistance);
        telemetry.addData("turn angle: ", turnDistance);
        telemetry.update();

        //strafing
        if (strafeDistance > 0) {

            // move left (right since backwards)
            direction =  1;

        } else if (strafeDistance < 0) {

            // move right (left since backwards)
            direction = -1;

        }

        return direction;

    }

    private void telemetryAprilTag() {

        List<AprilTagDetection> currentDetections = aprilTag.getDetections();
        telemetry.addData("# AprilTags Detected", currentDetections.size());

        // Step through the list of detections and display info for each one.
        for (AprilTagDetection detection : currentDetections) {
            if (detection.metadata != null) {
                telemetry.addLine(String.format("\n==== (ID %d) %s", detection.id, detection.metadata.name));
                telemetry.addLine(String.format("XYZ %6.1f %6.1f %6.1f  (inch)", detection.ftcPose.x, detection.ftcPose.y, detection.ftcPose.z));
                telemetry.addLine(String.format("PRY %6.1f %6.1f %6.1f  (deg)", detection.ftcPose.pitch, detection.ftcPose.roll, detection.ftcPose.yaw));
                telemetry.addLine(String.format("RBE %6.1f %6.1f %6.1f  (inch, deg, deg)", detection.ftcPose.range, detection.ftcPose.bearing, detection.ftcPose.elevation));
            } else {
                telemetry.addLine(String.format("\n==== (ID %d) Unknown", detection.id));
                telemetry.addLine(String.format("Center %6.0f %6.0f   (pixels)", detection.center.x, detection.center.y));
            }
        }   // end for() loop

        // Add "key" information to telemetry
        telemetry.addLine("\nkey:\nXYZ = X (Right), Y (Forward), Z (Up) dist.");
        telemetry.addLine("PRY = Pitch, Roll & Yaw (XYZ Rotation)");
        telemetry.addLine("RBE = Range, Bearing & Elevation");

    }
}
