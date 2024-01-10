package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.TouchSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@TeleOp
@Disabled
public class SensorOutput extends LinearOpMode {

    public ColorSensor colorSensor;
    public DistanceSensor liftDistance;
    public TouchSensor touchSensor;

    public DcMotorEx crane;


    boolean isTouched = false;


    @Override
    public void runOpMode() throws InterruptedException {

        colorSensor = hardwareMap.get(ColorSensor.class, "Color");
        liftDistance = hardwareMap.get(DistanceSensor.class, "Distance");
        touchSensor = hardwareMap.get(TouchSensor.class, "Touch");

        crane = hardwareMap.get(DcMotorEx.class, "Crane");



        while (!isStarted()) {
            telemetry.addData("lift distance: ", liftDistance.getDistance(DistanceUnit.INCH));

            telemetry.addData("color values alpha: ", colorSensor.alpha());
            telemetry.addData("color values blue: ", colorSensor.blue());
            telemetry.addData("color values red: ", colorSensor.red());

            telemetry.update();

        }

        waitForStart();

        while (opModeIsActive()) {

            telemetry.addData("lift distance: ", liftDistance.getDistance(DistanceUnit.INCH));

            telemetry.addData("color values alpha: ", colorSensor.alpha());
            telemetry.addData("color values blue: ", colorSensor.blue());
            telemetry.addData("color values red: ", colorSensor.red());

            telemetry.update();

            if (touchSensor.isPressed() && gamepad2.left_stick_y > 0){
                isTouched = true;
                crane.setPower(0);
            } else if (gamepad2.left_stick_y < 0) {
                isTouched = false;
                crane.setPower(gamepad2.left_stick_y);
            }
            if(isTouched == false) {
                crane.setPower(gamepad2.left_stick_y);
            }

        }
    }
}
