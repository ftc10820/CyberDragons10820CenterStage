package org.firstinspires.ftc.teamcode;

import android.util.Size;

import com.acmerobotics.dashboard.config.Config;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
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

import java.util.List;
import java.util.concurrent.TimeUnit;


@Autonomous
@Config
public class States_RedOutside extends LinearOpMode {

    //private org.firstinspires.ftc.vision.apriltag.AprilTagDetection desiredTag = null;

    public SampleMecanumDrive drive;

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

    // sensors used for pixel intake
    private ColorSensor pixelDetectorRight;
    private ColorSensor pixelDetectorLeft;

    // distance sensor on the bucket to detect when bucket is close to backdrop
    private DistanceSensor distanceBucket;

    // touch sensor used with linear slide
    public TouchSensor touchCrane;
    // Color sensor on bucket
    public ColorSensor colorBucket;

    // Vision portal and vision processing pipelines

    private AprilTagProcessor aprilTag;

    private TfodProcessor tfod;

    private VisionPortal myVisionPortal;

    private VisionPortal visionPortal;

    int zone = 1;

    int craneMax = 4500;


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
    private AprilTagDetection desiredTag = null;     // Used to hold the data for a detected AprilTag

    //april tag adjustment variables

    public static int aprilTagZone;

    public static double currentBearing = 10.0;
    public static double currentRange = 10.0;

    public static double currentYaw = 10.0;

    public static double desiredYaw = 10.0;

    public static double strafeAngleCorrection = 10.0;

    public static double strafeDistanceCorrection = 6.0;

    public static double strafeDistance = 5.0;
    public static double turnDistance = 0;

    public double slowerVelocity = 20.0;

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


    @Override
    public void runOpMode() throws InterruptedException {

        initialize();
        initIntakePlatform();
        closeRightIntake();
        closeLeftIntake();

        vision = new VisionSubsystem(hardwareMap);
        vision.setAlliance("red");

        drive = new SampleMecanumDrive(hardwareMap);

        //Starting the robot at the bottom left (blue auto)
        Pose2d startPose = new Pose2d(-36, -60, Math.toRadians(270));
        drive.setPoseEstimate(startPose);

        Trajectory zone1_1 = drive.trajectoryBuilder(startPose)
                .lineToLinearHeading(new Pose2d(-40,-34, Math.toRadians(0)))
                .build();

        Trajectory zone1_2 = drive.trajectoryBuilder(zone1_1.end())
                .forward(10.0)
                .build();

        Trajectory zone1_3 = drive.trajectoryBuilder(zone1_2.end())
                .back(10.0)
                .build();

        Trajectory zone1_4 = drive.trajectoryBuilder(zone1_3.end().plus(new Pose2d(0,0, Math.toRadians(180))))
                .strafeRight(24.0)
                .build();

        Trajectory zone1_5 = drive.trajectoryBuilder(zone1_4.end())
                .lineToLinearHeading(new Pose2d(-12,-6, Math.toRadians(180)))
                .build();

        Trajectory zone2_1 = drive.trajectoryBuilder(startPose)
                .back(44.0)
                .build();

        Trajectory zone2_2 = drive.trajectoryBuilder(zone2_1.end())
                .back(12.0)
                .build();

        Trajectory zone2_3 = drive.trajectoryBuilder(zone2_2.end().plus(new Pose2d(0,0, Math.toRadians(-90))))
                .lineToLinearHeading(new Pose2d(-12,-6, Math.toRadians(180)))
                .build();

        Trajectory zone3_1 = drive.trajectoryBuilder(startPose)
                .lineToLinearHeading(new Pose2d(-40,-24, Math.toRadians(180)))
                .build();

        Trajectory zone3_2 = drive.trajectoryBuilder(zone3_1.end())
                .forward(3.0)
                .build();

        Trajectory zone3_3 = drive.trajectoryBuilder(zone3_2.end())
                .back(6.0)
                .build();

        Trajectory zone3_4 = drive.trajectoryBuilder(zone3_3.end())
                .strafeRight(20.0)
                .build();

        Trajectory zone3_5 = drive.trajectoryBuilder(zone3_4.end())
                .lineToLinearHeading(new Pose2d(-12,-6, Math.toRadians(180)))
                .build();

        Trajectory backdrop_1 = drive.trajectoryBuilder(new Pose2d(-12,-6, Math.toRadians(180)))
                .lineToLinearHeading(new Pose2d(30,-6, Math.toRadians(180)))
                .build();

        Trajectory backdrop_2 = drive.trajectoryBuilder(backdrop_1.end())
                .lineToLinearHeading(new Pose2d(44,-40, Math.toRadians(180)))

                .build();

        while (!isStarted()) {
            vision.elementDetection(telemetry);
            vision.returnDistance(telemetry);

            telemetry.update();

        }

        zone = vision.zone;
        aprilTagZone = 3 + vision.zone;

        // for testing at home only
        // aprilTagZone = vision.zone;

        waitForStart();


        if (opModeIsActive()) {

            eTime2.reset();

            initAprilTag();

            if (zone == 3) {

                drive.followTrajectory(zone1_1);
                drive.followTrajectory(zone1_2);
                openLeftIntake();
                drive.followTrajectory(zone1_3);
                drive.turn(Math.toRadians(200));
                drive.followTrajectory(zone1_4);
                drive.followTrajectory(zone1_5);

            } else if (zone == 2) {

                drive.followTrajectory(zone2_1);
                openLeftIntake();
                drive.followTrajectory(zone2_2);
                drive.turn(Math.toRadians(-90));
                drive.followTrajectory(zone2_3);

            } else {

                drive.followTrajectory(zone3_1);
                drive.followTrajectory(zone3_2);
                openLeftIntake();
                drive.followTrajectory(zone3_3);
                drive.followTrajectory(zone3_4);
                drive.followTrajectory(zone3_5);

            }

            //pausing in middle to let alliance partner score
            while (eTime2.milliseconds() < 18000) {

            }

            drive.followTrajectory(backdrop_1);
            drive.followTrajectory(backdrop_2);

            //april tag paths
            Pose2d aprilTagStartPose = new Pose2d(0, 0, Math.toRadians(180));
            drive.setPoseEstimate(aprilTagStartPose);
            
            telemetryAprilTag();
            telemetry.update();

            aprilTagAdjustment();

            //april tag alignment
            drive.turn(turnDistance);

            Trajectory strafeAprilTag = drive.trajectoryBuilder(aprilTagStartPose.plus(new Pose2d(0,0, Math.toRadians(turnDistance))))
                    .strafeRight(strafeDistance)
                    .build();

            drive.followTrajectory(strafeAprilTag);

            visionPortal.setProcessorEnabled(aprilTag, false);

            if (desiredTag != null) {

                setCranePos(0.7); // in between low and medium
                sleep(1500) ;
                extendCraneUseColorSensorVelocity(2500, 5000, 600, 3000);
                setCranePos(CRANE_ANGLE_MEDIUM);
                sleep(1000) ;
                retractCraneHomeVelocity(CRANE_MAX_VELOCITY, 2000);
                positionCraneBase();
                retractCraneHomeVelocity(CRANE_MAX_VELOCITY, 2000);
                /*
                positionCraneMedium();
                sleep(1000) ;
                extendCraneUseColorSensorVelocity(CRANE_MAX_VELOCITY, 5000, 600, 3000);

                liftCraneSlightly(0.2);
                sleep(500);
                retractCraneHomeVelocity(CRANE_MAX_VELOCITY, 2000);
                positionCraneBase();
                retractCraneHomeVelocity(CRANE_MAX_VELOCITY, 2000);

                 */
            }

        }

    }

    public void initialize() {

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


        //sensors
        touchCrane = hardwareMap.get(TouchSensor.class, "Touch");
        colorBucket = hardwareMap.get(ColorSensor.class, "Color");

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
        intakeRight.setPosition(0);
    }

    void closeRightIntake() {
        intakeRight.setPosition(1.0);
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

    void extendCraneVelocity(double vel) {
        crane.setVelocity(vel);
    }
    int extendCraneUseDistanceSensorVelocity(double vel, int timeout_milli, int backdrop_dist_cm, int slow_time) {
        // extend crane till a timeout value or till the sensor detects closeness to backdrop
        // simpler function than when color sensor is used
        eTime1.reset();
        crane.setVelocity(vel);
        double distval = 0.0 ;

        while((distval = distanceBucket.getDistance(DistanceUnit.CM)) > backdrop_dist_cm) //
        {
            if ((crane.getCurrentPosition() > CRANE_MAX_ENCODER_VAL) || (eTime1.milliseconds() > timeout_milli))
                break ;

        }
        stopCrane();
        sleep(200);

        telemetry.addData("Distance val at stop:", distval );
        telemetry.update() ;


        // reversing to slow down motor
        crane.setVelocity(-vel*0.15);
        sleep(200) ;
        stopCrane();

        if ((crane.getCurrentPosition() < CRANE_MAX_ENCODER_VAL) ) {

            crane.setVelocity(vel*0.15);
            int ttime = slow_time ;
            // for the highest angle; increase time
            if (craneAngle.getPosition() > 0.8)
                ttime = ttime + 500;
            eTime1.reset();
            while(eTime1.milliseconds() < ttime) {
            }
            stopCrane();

        }

        return 0 ;

    }
    int extendCraneUseColorSensorVelocity(double vel, int timeout_milli, int backdrop_color_val, int slow_time) {
        // extend crane till given timeout value or till the sensor detects proximity to backdrop based on color sensor
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

        }
        stopCrane();
        sleep(200);

        telemetry.addData("Color val at stop:", colorVal + "currentVal " + colorBucket.red());
        telemetry.update() ;

        // reversing to slow down motor
        // check if this is useful
        retractCraneVelocity(vel*0.15);
        sleep(200) ;
        stopCrane();

        if ((crane.getCurrentPosition() < CRANE_MAX_ENCODER_VAL) ) {

            crane.setVelocity(vel*0.15);
            int ttime = slow_time ;
            // for the highest angle; increase time
            if (craneAngle.getPosition() > 0.8)
                ttime = ttime + 500;
            eTime1.reset();
            while((eTime1.milliseconds() < ttime) && (colorBucket.red() < backdrop_color_val) && (crane.getCurrentPosition() < CRANE_MAX_ENCODER_VAL)) {
            }
            stopCrane();

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

    void liftCraneSlightly(double incr) {
        craneAngle.setPosition(craneAngle.getPosition() + incr);
    }

    void aprilTagAdjustment() {

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

            return;
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
        strafeDistance = currentRange * (Math.sin((Math.toRadians(currentBearing + strafeAngleCorrection))));
        turnDistance = Math.toRadians(-1 * (desiredYaw - currentYaw));


/* TODO: Coach Weston: adjustment should still be 6 but we need to make sure it's
         aprilTagZone - desiredTag.id or is it desiredTag.id - aprilTagZone?
*/
        // correction if detected tag isnt desired zone
        strafeDistance += strafeDistanceCorrection * (desiredTag.id - aprilTagZone);
/*
 TODO: Coach Weston: We may want to subtract a final 2-4 inches since we want the camera
       about 2 inches to the left of the bucket.
*/

        telemetry.addData("correction: ", desiredTag.id - aprilTagZone);
        telemetry.addData("strafeDistance: ", strafeDistance);
        telemetry.addData("turn angle: ", turnDistance);
        telemetry.update();
        sleep(250);

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
