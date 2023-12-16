package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;
import org.firstinspires.ftc.vision.VisionPortal;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

@Autonomous
public class ScrimmageAutonomousBlueRight extends LinearOpMode {

    public DcMotorEx frontLeft;
    public DcMotorEx frontRight;
    public DcMotorEx backLeft;
    public DcMotorEx backRight;

    public DcMotorEx crane;
    public Servo indexer;
    public Servo intakeLeft;
    public Servo intakeRight;
    public Servo craneAngle;
    public Servo ramp;

    public ColorSensor colorSensor;
    public TouchSensor touchSensor;

    public AprilTagProcessor aprilTag;

    public VisionPortal visionPortal;

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
            initAprilTag();
            moveBackward(500,0.75);
            moveBackward(0.1);
            int maxColor = 0;
            while (colorSensor.blue() < 1900) {

                if (colorSensor.blue() > maxColor) {

                    maxColor = colorSensor.blue();
                }

                telemetry.addData("color value", maxColor);
                telemetry.update();

            }
            stopAllWheels();


            // code for auto 1
            /*if (zone == 1) {

                moveForward(250,0.5);
                turnLeft(425, 1.0);

                moveBackward(0.1);
                while (colorSensor.blue() < 1900) {

                }
                stopAllWheels();
                moveBackward(500, 0.5);
                intakeLeft.setPosition(1);
                sleep(500);

            } else if (zone == 2) {

                moveBackward(500, 0.5);
                intakeLeft.setPosition(1);
                sleep(500);

                moveBackward(700, 0.25);


                turnLeft(425, 1.0);

            } else {

                moveForward(150,0.5);
                turnLeft(425, 1.0);
                moveForward(250, 0.5);
                intakeLeft.setPosition(1);
                sleep(500);

            }*/


            /*
            // code for auto 1
            moveBackward(1250, 0.75);
            moveBackward(0.1);
            while (colorSensor.blue() < 1900) {

            }
            stopAllWheels();

             */
            if (zone == 1) {

                moveBackward(600, 0.75);
                turnLeft(450, 1.0);
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
            // code for auto 2
            moveBackward(600, 0.5);
            if (zone == 2){
                moveRight(400, .5);
            }
            if (zone == 3) {
                moveBackward(0.1);
                while (colorSensor.blue() < 1900) {}
            }
            stopAllWheels();
            if(zone == 2 || zone == 1){
                List<AprilTagDetection> currentDetections;
                boolean apirlTagFound = false;
                do {
                    currentDetections = aprilTag.getDetections();
                    for(AprilTagDetection detection : currentDetections){
                        if(detection.id == zone){
                            apirlTagFound = true;
                        }
                    }
                    telemetry.addData("X:", getDistanceToAprilTag(zone, true));
                    telemetry.update();
                    moveRight(25, .25);
                    //sleep(50);

                   /* if (currentDetections.size() > 0 && currentDetections.id){
                        apirlTagFound = true;
                    }*/
                }while(!apirlTagFound);
                stopAllWheels();
            }
            do {
                telemetry.addData("X:", getDistanceToAprilTag(zone, true));
                telemetry.update();
                if(getDistanceToAprilTag(zone, true) != 0){
                    if(getDistanceToAprilTag(zone, true) > 4){
                        moveLeft(50, .25);
                    } else if (getDistanceToAprilTag(zone, true) < 3) {
                        moveRight(50,.25);
                    }
                    else  {
                        break;
                    }
                }
                //y thirteen
            }while (getDistanceToAprilTag(zone, true) > 4 || getDistanceToAprilTag(zone, true) < 3);

            do {
                if(getDistanceToAprilTag(zone, false) != 0){
                    moveBackward(50, .25);
                }
                else {}
                telemetry.addData("Y:", getDistanceToAprilTag(zone, false));
                telemetry.update();
            }while ((getDistanceToAprilTag(zone, false) > 13) && zone == 2);
            //sleep(50);
            stopAllWheels();
            if(zone == 3 || zone == 2){
                moveBackward(250,.5);
            }
            craneAngle.setPosition(.6);
            sleep(500);

            crane.setPower(-1);
            sleep(3500);
            crane.setPower(0);
            sleep(250);
            moveForward(250,.5);
            crane.setPower(1);
            while(!touchSensor.isPressed()){

            }
            crane.setPower(0);
            craneAngle.setPosition(0);
            moveRight(500,.75);
            if(zone == 3 || zone == 2){
                moveRight(500, .75);
            }
            if(zone == 3){
                moveBackward(125,.5);
            }
            else {
                moveBackward(250, .5);
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
        colorSensor = hardwareMap.get(ColorSensor.class, "Color");



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

    void moveBackward(double speed) throws InterruptedException {
        frontLeft.setPower(speed);
        frontRight.setPower(speed);
        backLeft.setPower(speed);
        backRight.setPower(speed);

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

    void moveForward(double speed) throws InterruptedException {
        frontLeft.setPower(-speed);
        frontRight.setPower(-speed);
        backLeft.setPower(-speed);
        backRight.setPower(-speed);

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

    void stopAllWheels() throws InterruptedException {

        frontLeft.setPower(0);
        frontRight.setPower(0);
        backLeft.setPower(0);
        backRight.setPower(0);

    }

    private void initAprilTag() {

        // Create the AprilTag processor.
        aprilTag = new AprilTagProcessor.Builder()
                //.setDrawAxes(false)
                //.setDrawCubeProjection(false)
                //.setDrawTagOutline(true)
                //.setTagFamily(AprilTagProcessor.TagFamily.TAG_36h11)
                //.setTagLibrary(AprilTagGameDatabase.getCenterStageTagLibrary())
                //.setOutputUnits(DistanceUnit.INCH, AngleUnit.DEGREES)

                // == CAMERA CALIBRATION ==
                // If you do not manually specify calibration parameters, the SDK will attempt
                // to load a predefined calibration for your camera.
                //.setLensIntrinsics(578.272, 578.272, 402.145, 221.506)

                // ... these parameters are fx, fy, cx, cy.

                .build();

        // Create the vision portal by using a builder.
        VisionPortal.Builder builder = new VisionPortal.Builder();

        // Set the camera (webcam vs. built-in RC phone camera).
       builder.setCamera(hardwareMap.get(WebcamName.class, "Webcam 1"));


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
        builder.addProcessor(aprilTag);

        // Build the Vision Portal, using the above settings.
        visionPortal = builder.build();
        //Changes resolution (CyberDragons Change)
        aprilTag.setDecimation(1);
        // Disable or re-enable the aprilTag processor at any time.
        //visionPortal.setProcessorEnabled(aprilTag, true);

    }
    private double getDistanceToAprilTag(int zone, boolean findX){
        List<AprilTagDetection> currentDetections = aprilTag.getDetections();
        for (AprilTagDetection detection : currentDetections) {
            if(detection.id == zone){
                if(findX) {
                    return detection.ftcPose.x;
                }
                else {
                    return detection.ftcPose.y;
                }
            }
        }
        return 0.0;
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
