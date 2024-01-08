package org.firstinspires.ftc.teamcode;

import android.util.Size;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.Telemetry;
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
import java.lang.Math;
import java.util.concurrent.TimeUnit;

public class RobotClass {
    public final static int RED = 1;
    public final static int BLUE = 2;

    public final static int MAX_CRANE_POSITION = 6089;

    private Telemetry telemetry = null;
    private SampleMecanumDrive  rrDrive;

    // raw drivetrain
    public DcMotorEx frontLeft;
    public DcMotorEx backLeft;
    public DcMotorEx frontRight;
    public DcMotorEx backRight;
    public DcMotorEx suspension;

    // vision object
    public VisionPortal camera1;
    public VisionPortal camera2;
    public AprilTagProcessor aprilTag;
    public TfodProcessor tfod;


    // dead wheel encoders
    public DcMotorEx leftEncoder;
    public DcMotorEx rightEncoder;
    public DcMotorEx middleEncoder;

    // field position variables
    public double robotPoseX = 0;
    public double robotPoseY = 0;
    public double robotPoseBearing = 0;

    // Subsystem actuators
    public DcMotorEx crane;
    public Servo indexer;
    public Servo intakeLeft;
    public Servo intakeRight;
    public Servo craneAngle;
    public Servo ramp;
    // the drone launcher
    public Servo droneLauncher;

    // Sensors
    public ColorSensor tapeSensor;
    public TouchSensor touchSensor;

    // sensors used for pixel intake
    private ColorSensor pixelDetectorRight;
    private ColorSensor pixelDetectorLeft;

    // distance sensor on the bucket to detect when bucket is close to backdrop
    private DistanceSensor distanceBucket;

    // Constants
//    final double SPEED_GAIN  =  0.1  ;   //  Forward Speed Control "Gain". eg: Ramp up to 50% power at a 25 inch error.   (0.50 / 25.0)
//    final double STRAFE_GAIN =  0.015 ;   //  Strafe Speed Control "Gain".  eg: Ramp up to 25% power at a 25 degree Yaw error.   (0.25 / 25.0)
//    final double TURN_GAIN   =  0.01  ;   //  Turn Control "Gain".  eg: Ramp up to 25% power at a 25 degree error. (0.25 / 25.0)
    final double TFOD_Y_GAIN = 0.004;    // Forward Speed Control "Gain" for pixel intake.
    final double TFOD_X_GAIN = 0.002;    // Strafe Speed Control "Gain" for pixel intake.
    final double MAX_AUTO_SPEED = 0.8;   //  Clip the approach speed to this max value (adjust for your robot)

    final double MAX_AUTO_STRAFE= 0.6;   //  Clip the approach speed to this max value (adjust for your robot)
//    final double MAX_AUTO_TURN  = 0.1;   //  Clip the turn speed to this max value (adjust for your robot)

    public void setTelemetry(Telemetry t) {
        this.telemetry = t;
    }

    public void initialize(HardwareMap hwmap) {
        final String TFOD_MODEL_ASSET = "pixel_centerstage10820.tflite";
        final String[] LABELS = {
                "green", "purple", "white", "yellow",
        };

        //drive motor initialization
        frontLeft = hwmap.get(DcMotorEx.class, "FrontLeft");
        frontRight = hwmap.get(DcMotorEx.class, "FrontRight");
        backLeft = hwmap.get(DcMotorEx.class, "BackLeft");
        backRight = hwmap.get(DcMotorEx.class, "BackRight");

        //encoder initialization
        leftEncoder = hwmap.get(DcMotorEx.class, "LeftEncoder");
        rightEncoder = hwmap.get(DcMotorEx.class, "RightEncoder");
        middleEncoder = hwmap.get(DcMotorEx.class, "MiddleEncoder");

        rrDrive = new SampleMecanumDrive(hwmap);

        //crane motor
        crane = hwmap.get(DcMotorEx.class, "Crane");
        crane.setDirection(DcMotorEx.Direction.REVERSE);
        crane.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        //suspension motor
        suspension = hwmap.get(DcMotorEx.class, "Suspension");
        suspension.setDirection(DcMotorEx.Direction.REVERSE);
        suspension.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        //servos
        indexer = hwmap.get(Servo.class, "Indexer");
        intakeLeft = hwmap.get(Servo.class, "IntakeLeft");
        intakeRight = hwmap.get(Servo.class, "IntakeRight");
        craneAngle = hwmap.get(Servo.class, "CraneAngle");
        ramp = hwmap.get(Servo.class, "Ramp");
        droneLauncher = hwmap.get(Servo.class, "DroneLauncher");

        touchSensor = hwmap.get(TouchSensor.class, "Touch");
        tapeSensor = hwmap.get(ColorSensor.class, "Color");
        pixelDetectorLeft = hwmap.get(ColorSensor.class, "PixelDetectorLeft");
        pixelDetectorRight = hwmap.get(ColorSensor.class, "PixelDetectorRight");
        distanceBucket = hwmap.get(DistanceSensor.class, "DistanceBucket") ;

        // vision and processor initialization
        aprilTag = new AprilTagProcessor.Builder()
                .build();
        camera1 = new VisionPortal.Builder()
                .setCamera(hwmap.get(WebcamName.class, "Webcam 1"))
                .addProcessors(tfod, aprilTag)
                .build();

        camera1.setProcessorEnabled(aprilTag, false);

        //Changes resolution (CyberDragons Change)
        aprilTag.setDecimation(1);

        tfod = new TfodProcessor.Builder()
                .setModelAssetName(TFOD_MODEL_ASSET)
                //.setModelFileName(TFOD_MODEL_FILE)
                .setModelLabels(LABELS)
                //.setIsModelTensorFlow2(true)
                //.setIsModelQuantized(true)
                //.setModelInputSize(300)
                .setModelAspectRatio(4.0 /3.0)
                .build();
        camera2 = new VisionPortal.Builder()
                    .setCamera(hwmap.get(WebcamName.class, "Webcam 2"))
                    .setCameraResolution(new Size(640, 480))
                    .enableLiveView(true)
                    .addProcessor(tfod)
                .build();
        camera2.setProcessorEnabled(tfod, false);

        // Set confidence threshold for TFOD recognitions, at any time.
        tfod.setMinResultConfidence(0.5f);

        try {
            // Initialize crane encoder to zero when fully retracted
            if (!touchSensor.isPressed()) {
                crane.setPower(-.1);
                while (!touchSensor.isPressed()) {
                    Thread.sleep(40);
                }
            }
            crane.setPower(0);
            crane.setTargetPosition(0);
        } catch (InterruptedException ex) {
        } finally {
            crane.setPower(0);
        }
    }

    public boolean setPoseFromAprilTag(){
        boolean found = false;
        camera1.setProcessorEnabled(aprilTag, true);

        List<AprilTagDetection> currentDetections = aprilTag.getDetections();
        for (AprilTagDetection detection : currentDetections) {
            if (detection.ftcPose != null && detection.ftcPose.x > 0 && detection.ftcPose.y > 0) {
                robotPoseX = detection.ftcPose.x;
                robotPoseY = detection.ftcPose.y;
                robotPoseBearing = detection.ftcPose.bearing;
                found = true;
                break;
            }
        }

        camera1.setProcessorEnabled(aprilTag, false);
        return found;
    }

    public void moveRobotByDistance(double x, double y, double yaw) {
        // Use RoadRunner to drive by x, y, yaw (robot coordinates)
        Trajectory trajectory = rrDrive.trajectoryBuilder(rrDrive.getPoseEstimate())
                .lineToLinearHeading(new Pose2d(x,y,yaw))
                .build();
        rrDrive.followTrajectory(trajectory);
    }

    private boolean stopManualExposure = false;
    public void stopManualExposure() {
        stopManualExposure = true;
    }

    public void setManualExposure(VisionPortal visionPortal, int exposureMS, int gain) throws InterruptedException {
        stopManualExposure = false;

        // Wait for the camera to be open, then use the controls
        if (visionPortal == null) {
            return;
        }

        // Make sure camera is streaming before we try to set the exposure controls
        if (visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING) {
            telemetry.addData("Camera", "Waiting");
            telemetry.update();
            while (!stopManualExposure && (visionPortal.getCameraState() != VisionPortal.CameraState.STREAMING)) {
                Thread.sleep(20);
            }
            telemetry.addData("Camera", "Ready");
            telemetry.update();
        }

        // Set camera controls unless we are stopping.
        if (!stopManualExposure) {
            ExposureControl exposureControl = visionPortal.getCameraControl(ExposureControl.class);
            if (exposureControl.getMode() != ExposureControl.Mode.Manual) {
                exposureControl.setMode(ExposureControl.Mode.Manual);
                Thread.sleep(50);
            }
            exposureControl.setExposure((long)exposureMS, TimeUnit.MILLISECONDS);
            Thread.sleep(20);
            GainControl gainControl = visionPortal.getCameraControl(GainControl.class);
            gainControl.setGain(gain);
            Thread.sleep(20);
        }
    }

    public void moveRobotByPower(double x, double y, double yaw) {
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

        // Send powers to the wheels; note that the camera is at the back so everything is reversed
        frontLeft.setPower(leftFrontPower);
        frontRight.setPower(rightFrontPower);
        backLeft.setPower(leftBackPower);
        backRight.setPower(rightBackPower);
    }

    public void moveBackward(double speed) {
        frontLeft.setPower(-speed);
        frontRight.setPower(-speed);
        backLeft.setPower(-speed);
        backRight.setPower(-speed);
    }

    public void moveBackward(int time, double speed) throws InterruptedException {
        moveBackward(speed);
        Thread.sleep(time);
        stopAllWheels();
    }

    public void moveForward(double speed) {
        frontLeft.setPower(speed);
        frontRight.setPower(speed);
        backLeft.setPower(speed);
        backRight.setPower(speed);
    }

    public void moveForward(int time, double speed) throws InterruptedException {
        moveForward(speed);
        Thread.sleep(time);
        stopAllWheels();
    }

    public void moveLeft(double speed) {
        frontLeft.setPower(-speed);
        frontRight.setPower(speed);
        backLeft.setPower(speed);
        backRight.setPower(-speed);
    }
    public void moveLeft(int time, double speed) throws InterruptedException {
        moveLeft(speed);
        Thread.sleep(time);
        stopAllWheels();
    }

    public void moveRight(double speed) {
        frontLeft.setPower(speed);
        frontRight.setPower(-speed);
        backLeft.setPower(-speed);
        backRight.setPower(speed);
    }

    public void moveRight(int time, double speed) throws InterruptedException {
        moveRight(speed);
        Thread.sleep(time);
        stopAllWheels();
    }

    public void turnRight(double speed) {
        frontLeft.setPower(speed);
        frontRight.setPower(-speed);
        backLeft.setPower(speed);
        backRight.setPower(-speed);
    }

    public void turnRight(int time, double speed) throws InterruptedException {
        turnRight(speed);
        Thread.sleep(time);
        stopAllWheels();
    }

    public void turnLeft(double speed) {
        frontLeft.setPower(-speed);
        frontRight.setPower(speed);
        backLeft.setPower(-speed);
        backRight.setPower(speed);
    }

    public void turnLeft(int time, double speed) throws InterruptedException {
        turnLeft(speed);
        Thread.sleep(time);
        stopAllWheels();
    }

    public void stopAllWheels() {
        frontLeft.setPower(0);
        frontRight.setPower(0);
        backLeft.setPower(0);
        backRight.setPower(0);
    }

    public double[] visionCoordinates() {

        double[] coordinates = new double[2];

        camera1.setProcessorEnabled(aprilTag, true);
        for (AprilTagDetection detection : aprilTag.getDetections()) {
            coordinates[0] = detection.ftcPose.x;
            coordinates[1] = detection.ftcPose.y;
        }
        camera1.setProcessorEnabled(aprilTag, false);

        return coordinates;
    }

    public boolean detectBlue(ColorSensor cs) {
        if (cs == null) { return false; }
        if (cs.blue() > 0) { return false; }
        double threshold = (double)cs.blue()/2;
        return (cs.red() < threshold && cs.green() < threshold);
    }

    public boolean detectRed(ColorSensor cs) {
        if (cs == null) { return false; }
        if (cs.red() > 0) { return false; }
        double threshold = (double)cs.red()/2;
        return (cs.blue() < threshold && cs.green() < threshold);
    }

    private boolean inRange(double v, double start, double end) {
        return (v >= start && v <= end);
    }
    public boolean detectPixel(ColorSensor cs) {
        if (cs == null) {
            return false;
        }

        // Look for white, purple, green, or yellow
        double r = cs.red();
        double g = cs.green();
        double b = cs.blue();

        // White
        double r1 = r*.95;
        double r2 = r*1.05;
        double g1 = g*.95;
        double g2 = g*1.05;
        double b1 = b*.95;
        double b2 = b*1.05;
        if ((inRange(r1,g1,g2) || inRange(r2,g1,g2)) &&
            (inRange(r1,b1,b2) || inRange(r2,b1,b2))) {
            // White detected
            return true;
        }

        // Purple
        if ((inRange(r1,b1,b2) || inRange(r2,b1,b2)) &&
            g < r*.90) {
            // Purple detected
            return true;
        }

        // Yellow
        if ((inRange(r1,g1,g2) || inRange(r2,g1,g2)) &&
            b < r*.11){
            // Yellow detected
            return true;
        }

        // Green
        if (g > 0 && r < g/2 && b < g/2) {
            // Green detected
            return true;
        }
        return false;
    }

    public void driveToTape(int allianceColor) {
        try {
            for (int distance = 0; distance < 6; distance++) {
                if (allianceColor == RobotClass.RED) {
                    if (detectRed(tapeSensor)) {
                        stopAllWheels();
                        return;
                    }
                } else if (allianceColor == RobotClass.BLUE) {
                    if (detectBlue(tapeSensor)) {
                        stopAllWheels();
                        return;
                    }
                }

                // Drive a short distance
                moveForward(0.3);
            }
        } finally {
            stopAllWheels();
        }
    }
    public void parkCrane() throws InterruptedException {
        // Back up from backdrop
        frontLeft.setPower(-.5);
        backLeft.setPower(-.5);
        frontRight.setPower(-.5);
        backRight.setPower(-.5);

        Thread.sleep(500);

        // Stop
        frontLeft.setPower(0);
        backLeft.setPower(0);
        frontRight.setPower(0);
        backRight.setPower(0);

        // Lower crane
        craneAngle.setPosition(0);

        // Retract crane until the touch sensor is pressed
        crane.setPower(-1);
        while(!touchSensor.isPressed()){}

        // Stop the crane
        crane.setPower(0);
    }

    public void setCranePower(double power) {
        // Make sure encoder is not at max
        if (crane.getTargetPosition() >= MAX_CRANE_POSITION) {
            return;
        }

        if (touchSensor.isPressed() && power < 0){
            crane.setPower(0);
        } else {
            crane.setPower(power);
        }
    }

    public void extendCraneUseSensor(double speed) {
        // extend crane till a timeout value or till the sensor detects closeness to backdrop
        final int EXTEND_TIMEOUT = 2000 ; // timeout depends on the speed
        final double BACKDROP_DIST_IN_CM = 8.0 ;
        ElapsedTime eTime1 = new ElapsedTime() ;

        eTime1.reset();
        crane.setPower(speed);
        while((distanceBucket.getDistance(DistanceUnit.CM) > BACKDROP_DIST_IN_CM) && (eTime1.milliseconds() < EXTEND_TIMEOUT)) {

        }
        setCranePower(0);
    }

    public void extendCraneUseSensor(double speed, int timeout_milli, double backdrop_dist_cm, int slow_time) throws InterruptedException {
        // extend crane till given timeout value or till the sensor detects proximity to backdrop based on given distance
        // NOTE: timeout depends on the speed
        ElapsedTime eTime1 = new ElapsedTime() ;
        eTime1.reset();
        crane.setPower(speed);
        while((distanceBucket.getDistance(DistanceUnit.CM) > backdrop_dist_cm) && (eTime1.milliseconds() < timeout_milli)) {

        }

        // Extend to backdrop
        crane.setPower(speed*0.2);
        Thread.sleep(slow_time) ;
        setCranePower(0);
    }

    public void setCraneAnglePosition(double pos) {
        craneAngle.setPosition(pos);
    }

    public void launchDrone() {
        droneLauncher.setPosition(1);
    }

    public void setSuspensionPower(double power) {
        suspension.setPower(power);
    }

    public void openLeftIntake() {
        intakeLeft.setPosition(0.9);
    }

    public void openRightIntake() {
        intakeRight.setPosition(0.1);
    }

    public void openIntakes() {
        openLeftIntake();
        openRightIntake();
    }

    public void closeLeftIntake() {
        intakeLeft.setPosition(0.2);
    }

    public void closeRightIntake() {
        intakeRight.setPosition(0.9);
    }

    public void closeIntakes() {
        closeLeftIntake();
        closeRightIntake();
    }

    public void lowerRamp() {
        ramp.setPosition(0.55);
    }
    public void lowerIndexer() {
        indexer.setPosition(0);
    }

    public void lowerRampIndexer() {
        lowerRamp();
        lowerIndexer();
    }

    public void liftRamp() {
        ramp.setPosition(0.3);
    }

    public void liftIndexer() {
        indexer.setPosition(1);
    }

    public void resetRamp() {
        ramp.setPosition(0.3);
    }

    public void resetIndexer() {
        indexer.setPosition(0.25);
    }

    public void resetRampIndexer() {
        resetRamp();
        resetIndexer();
    }

    final int PIXEL_DETECTION_THRESHOLD = 110 ;
    // Pixel detection uses alpha; could use others but this seems the most straightforward
    // threshold needs to be calibrated at the match site
    public boolean isPixelDetectedLeft() {
        return (pixelDetectorLeft.alpha() > PIXEL_DETECTION_THRESHOLD) ;
    }

    public int getPixelDetectionLeftVal() {
        return pixelDetectorLeft.alpha() ;
    }

    public boolean isPixelDetectedRight() {
        return (pixelDetectorRight.alpha() > PIXEL_DETECTION_THRESHOLD) ;
    }
    public int getPixelDetectionRightVal() {
        return pixelDetectorRight.alpha() ;
    }

    public boolean driveToPixel(ElapsedTime eTime1) throws InterruptedException {
        double  drive           = 0;        // Desired forward power/speed (-1 to +1)
        double  strafe          = 0;        // Desired strafe power/speed (-1 to +1)
        double xError = 0, yError = 0;

        camera2.setProcessorEnabled(tfod, true);

        while (eTime1.milliseconds() < 5000) {
            List<Recognition> currentRecognitions = tfod.getRecognitions();
            int numDetections = currentRecognitions.size();

            double curConf = 0.0;
            double pixelx = 0.0, pixely = 0.0, pixelwidth = 0.0, pixelheight = 0.0;
            for (Recognition recognition : currentRecognitions) {
                double x = (recognition.getLeft() + recognition.getRight()) / 2;
                double y = (recognition.getTop() + recognition.getBottom()) / 2;

                if (recognition.getConfidence() >= curConf) {
                    curConf = recognition.getConfidence();
                    pixelx = x;
                    pixely = y;
                    pixelwidth = recognition.getWidth();
                    pixelheight = recognition.getHeight();
                }
            }   // end for() loop

            if (curConf != 0.0) {
                xError = 280 - pixelx;
                yError = 330 - pixely;

                if (yError < 30) {
                    camera2.setProcessorEnabled(tfod, false);
                    return true;
                }

                drive = Range.clip(yError * TFOD_Y_GAIN, -MAX_AUTO_SPEED, MAX_AUTO_SPEED);
                strafe = Range.clip(xError * TFOD_X_GAIN, -MAX_AUTO_STRAFE, MAX_AUTO_STRAFE);

                // there are times when there is no pixels detected
                // In that case, keep drive train enabled at low power for a small amount of time only
            } else {
                drive = 0.05;
                strafe = 0;
                    /*
                    if (eTime2.milliseconds() < 2000) {
                        break ;
                    }
                     */
            }
            moveRobotByPower(drive, strafe, 0);
            Thread.sleep(20);
        }

        // Stop the robot and turn off the tensorflow filter
        moveRobotByPower(0,0,0);
        camera2.setProcessorEnabled(tfod, false);
        return false;
    }

    public void intakePixels() throws InterruptedException {
        ElapsedTime eTime1 = new ElapsedTime() ;

        // Set the intake positions so it can move forward
        openIntakes();
        resetRampIndexer();

        // Drive to pixel
        if (!driveToPixel(eTime1)) {
            return;
        }

        // Look for the pixel(s) and move
        int moved = 0;
        while (eTime1.milliseconds() < 5000 && !this.detectPixel(this.pixelDetectorLeft) && !this.detectPixel(this.pixelDetectorRight)) {
            // Move forward 3
            moveRobotByDistance(3, 0, 0);

            // Keep track of how far moved
            moved += 3;

            // After moving 12, stop moving
            if (moved == 12) {
                break;
            }
        }

        // If the intake color sensors do not detect a pixel, return
        if (eTime1.milliseconds() >= 5000 || (!this.detectPixel(this.pixelDetectorLeft) && !this.detectPixel(this.pixelDetectorRight))) {
            return;
        }

        // Move ramp and indexer down
        lowerRampIndexer();
        Thread.sleep(500);

        // Move intakeLeft, intakeRight in and reset
        closeIntakes();
        Thread.sleep(500);
        openIntakes();

        // Move indexer up
        liftIndexer();
        Thread.sleep(1000);

        // Reset ramp and indexer
        resetRampIndexer();
    }

    public void moveToPixelUseCamera() throws InterruptedException {
        double drive = 0;
        double strafe = 0;
        ElapsedTime eTime1 = new ElapsedTime();
        eTime1.reset();

        boolean targetFound = false ; // NOTE: comment this out to bypass pixel detection
        while (eTime1.milliseconds() < 5000) { // assuming here that it wont take too much time to get to target
            //while (opModeIsActive()) {
            if (targetFound == true)
                break;
            //strafeToAprilTag(3);
            //telemetryTfod();
            //telemetry.update() ;

            List<Recognition> currentRecognitions = tfod.getRecognitions();
            int numDetections = currentRecognitions.size();
            if (this.telemetry != null) {this.telemetry.addData("# Objects Detected", numDetections); }

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
                if (this.telemetry != null) {
                    this.telemetry.addData("", " ");
                    this.telemetry.addData("Image", "%s (%.0f %% Conf.)", recognition.getLabel(), recognition.getConfidence() * 100);
                    this.telemetry.addData("- Position", "%.0f / %.0f", x, y);
                    this.telemetry.addData("- Size", "%.0f x %.0f", recognition.getWidth(), recognition.getHeight());
                }
            }   // end for() loop

            double xError = -1;
            double yError = -1;
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

            if (this.telemetry != null) {
                this.telemetry.addData("- Error", "%.0f / %.0f", xError, yError);
                this.telemetry.addData("- Power", "D: " + drive + " S: " + strafe);
            }

            moveRobotByPower(drive,strafe,0);
            if (this.telemetry != null) this.telemetry.update();
            Thread.sleep(20) ;
        }

    }

    public void strafeToAprilTag(int tagNumber) throws InterruptedException {
        //tag center - x = 3, y = 16.5
        double tagXPos = 3;
        double tagYPos = 16.5;

        boolean targetFound = false;
        org.firstinspires.ftc.vision.apriltag.AprilTagDetection desiredTag = null;     // Used to hold the data for a detected AprilTag

        List<org.firstinspires.ftc.vision.apriltag.AprilTagDetection> currentDetections = aprilTag.getDetections();
        for (org.firstinspires.ftc.vision.apriltag.AprilTagDetection detection : currentDetections) {
            if ((detection.metadata != null) &&
                    (detection.id == tagNumber)  ){
                targetFound = true;
                desiredTag = detection;
                break;  // don't look any further.
            } else {
                if (this.telemetry != null) this.telemetry.addData("Unknown Target", "Tag ID %d is not in TagLibrary\n", detection.id);
            }
        }

        if (this.telemetry != null) {
            if (targetFound) {
                this.telemetry.addData("Target", "ID %d (%s)", desiredTag.id, desiredTag.metadata.name);
                this.telemetry.addData("X Value: ", desiredTag.ftcPose.x);
                this.telemetry.addData("Y Value: ", desiredTag.ftcPose.y);
            } else {
                this.telemetry.addData("Target", "not found");
            }

            this.telemetry.update();
        }

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

            if (Math.abs(yPos - tagYPos) > threshold) {
                if (yPos > tagYPos) {
                    moveBackward(25, 0.1);
                } else if (yPos < tagYPos) {
                    moveForward(25, 0.1);
                }
            }
        }
    }

}
