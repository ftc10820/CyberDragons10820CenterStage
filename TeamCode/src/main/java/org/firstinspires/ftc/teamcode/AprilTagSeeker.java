package org.firstinspires.ftc.teamcode;

import android.graphics.Paint;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;
import org.firstinspires.ftc.robotcore.external.Telemetry;

import java.util.List;

public class AprilTagSeeker {

    public AprilTagSeeker(Telemetry telemetry) {
        //TODO: init the april tags
        this.telemetry = telemetry;
    }
    private AprilTagProcessor aprilTag;
    private AprilTagDetection desiredTag = null;

    private Telemetry telemetry = null;

    public boolean moveUntilTag(SampleMecanumDrive drive, int zone ) throws InterruptedException{
        return moveUntilTag(drive, zone, false);
    }
    public boolean moveUntilTag(SampleMecanumDrive drive, int zone, boolean isBoardRight) throws InterruptedException {

        //tag center - x = 3, y = 16.5
        double tagXPos = 3;
        double tagYPos = 16.5;

        boolean targetFound = false;
        boolean xAligned = false;
        desiredTag = null;

        //if(currentDetections == null  || currentDetections.isEmpty()){
       if( !strafeToTag(drive, isBoardRight, zone)){
           return false;
        }

        List<AprilTagDetection> currentDetections = aprilTag.getDetections();

        for (AprilTagDetection detection : currentDetections) {
            if ((detection.metadata != null) &&
                    (detection.id == zone)) {
                targetFound = true;
                desiredTag = detection;
                break;
            } else {
                telemetry.addData("Unknown Target", "Tag ID %d is not in TagLibrary\n", detection.id);
            }
        }

        if (targetFound) {
            telemetry.addData("Target", "ID %d (%s)", desiredTag.id, desiredTag.metadata.name);
            telemetry.addData("X Value: ", desiredTag.ftcPose.x);
            telemetry.addData("Y Value: ", desiredTag.ftcPose.y);
            telemetry.update();

        } else {
          telemetry.addLine("Tag no longer found");
          telemetry.update();
          return false;
        }



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

    public boolean strafeToTag(SampleMecanumDrive drive, boolean boardIsRight, int zone) throws InterruptedException{
        //TODO: we need an exit strategy for when to stop if no tag is found, maybe overall time
        List<AprilTagDetection> currentDetections = aprilTag.getDetections();

        if(boardIsRight){
           // strafe until you find any April Tag
            while(currentDetections == null || currentDetections.isEmpty()){
                drive.moveRight(50, 0.5);
                currentDetections = aprilTag.getDetections();
                telemetry.addLine(" moved right ");
                telemetry.update();
            }

            //check if you have found th correct April Tag
            while(! foundTag(currentDetections, zone)) {

                drive.moveRight(50, 0.25);
                currentDetections = aprilTag.getDetections();
                telemetry.addLine(" looking for correct tag to right ");
                telemetry.update();
                currentDetections = aprilTag.getDetections();
            }
            return true;

        }else {
            while (currentDetections == null || currentDetections.isEmpty()) {
                drive.moveLeft(10, 0.5);
                currentDetections = aprilTag.getDetections();
                telemetry.addLine(" moved left ");
                telemetry.update();
            }
            //check if you have found th correct April Tag
            while (!foundTag(currentDetections, zone)) {

                drive.moveLeft(50, 0.25);
                currentDetections = aprilTag.getDetections();
                telemetry.addLine(" looking for correct tag to right ");
                telemetry.update();
                currentDetections = aprilTag.getDetections();
            }
            return true;
        }
    }


    private boolean foundTag(List <AprilTagDetection> detections, int zone){
        if(detections != null && detections.isEmpty()){
            for (AprilTagDetection detection : detections) {
                if (detection.id == zone) {
                    return true;
                }
            }
        }
        return false;
    }
}
