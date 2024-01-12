package org.firstinspires.ftc.teamcode;

import android.graphics.Paint;

import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;

import java.util.List;

public class AprilTagSeeker {
    public static boolean moveUntilTag(SampleMecanumDrive drive, int zone ){
        return moveUntilTag(drive, zone, false);
    }
    public static boolean moveUntilTag(SampleMecanumDrive drive, int zone, boolean right){
        List<org.firstinspires.ftc.vision.apriltag.AprilTagDetection> currentDetections = aprilTag.getDetections();
        for (AprilTagDetection detection : currentDetections) {
            if(detection.id == zone){
                return true;
            }
        }
        return false;
    }
    public static void alignWithTag(){

    }
}
