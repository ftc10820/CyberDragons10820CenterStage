package org.firstinspires.ftc.teamcode;

import android.util.Size;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.robotcore.external.tfod.Recognition;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.vision.tfod.TfodProcessor;

import java.util.List;
import java.lang.Math;

public class RobotClass {

    // drivetrain
    public DcMotorEx frontLeft;
    public DcMotorEx backLeft;
    public DcMotorEx frontRight;
    public DcMotorEx backRight;

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
    public ColorSensor colorSensor;
    public TouchSensor touchSensor;

    // sensors used for pixel intake
    private ColorSensor pixelDetectorRight;
    private ColorSensor pixelDetectorLeft;

    // Constants
//    final double SPEED_GAIN  =  0.1  ;   //  Forward Speed Control "Gain". eg: Ramp up to 50% power at a 25 inch error.   (0.50 / 25.0)
//    final double STRAFE_GAIN =  0.015 ;   //  Strafe Speed Control "Gain".  eg: Ramp up to 25% power at a 25 degree Yaw error.   (0.25 / 25.0)
//    final double TURN_GAIN   =  0.01  ;   //  Turn Control "Gain".  eg: Ramp up to 25% power at a 25 degree error. (0.25 / 25.0)
    final double TFOD_Y_GAIN = 0.004;    // Forward Speed Control "Gain" for pixel intake.
    final double TFOD_X_GAIN = 0.002;    // Strafe Speed Control "Gain" for pixel intake.
    final double MAX_AUTO_SPEED = 0.8;   //  Clip the approach speed to this max value (adjust for your robot)

    final double MAX_AUTO_STRAFE= 0.6;   //  Clip the approach speed to this max value (adjust for your robot)
//    final double MAX_AUTO_TURN  = 0.1;   //  Clip the turn speed to this max value (adjust for your robot)

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

        //crane motor
        crane = hwmap.get(DcMotorEx.class, "Crane");

        //servos
        indexer = hwmap.get(Servo.class, "Indexer");
        intakeLeft = hwmap.get(Servo.class, "IntakeLeft");
        intakeRight = hwmap.get(Servo.class, "IntakeRight");
        craneAngle = hwmap.get(Servo.class, "CraneAngle");
        ramp = hwmap.get(Servo.class, "Ramp");
        droneLauncher = hwmap.get(Servo.class, "DroneLauncher");

        touchSensor = hwmap.get(TouchSensor.class, "Touch");
        colorSensor = hwmap.get(ColorSensor.class, "Color");
        pixelDetectorLeft = hwmap.get(ColorSensor.class, "PixelDetectorLeft");
        pixelDetectorRight = hwmap.get(ColorSensor.class, "PixelDetectorRight");

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
        // TODO: Use RoadRunner to drive by x, y, yaw (robot coordinates)
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
        double threshold = cs.blue()/2;
        return (cs.red() < threshold && cs.green() < threshold);
    }

    public boolean detectRed(ColorSensor cs) {
        if (cs == null) { return false; }
        if (cs.red() > 0) { return false; }
        double threshold = cs.red()/2;
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
        crane.setPower(1);
        while(!touchSensor.isPressed()){}

        // Stop the crane
        crane.setPower(0);
    }

    public void extendCrane(double power) {
        boolean isTouched = false;
        if (touchSensor.isPressed() && power > 0){
            isTouched = true;
            crane.setPower(0);
        } else if (power < 0) {
            isTouched = false;
            crane.setPower(power);
        }
        if(isTouched == false) {
            crane.setPower(power);
        }
    }

    public void setCraneAnglePosition(double pos) {
        craneAngle.setPosition(pos);
    }

    public void launchDrone() {
        droneLauncher.setPosition(1);
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

    public void intakesClose() {
        closeLeftIntake();
        closeRightIntake();
    }

    public void lowerRamp() {
        ramp.setPosition(0.45);
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
        intakesClose();
        Thread.sleep(500);
        openIntakes();

        // Move indexer up
        liftIndexer();
        Thread.sleep(1000);

        // Reset ramp and indexer
        resetRampIndexer();
    }

}
