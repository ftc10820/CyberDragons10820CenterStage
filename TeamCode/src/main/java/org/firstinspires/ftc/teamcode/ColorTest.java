package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;

@Autonomous
public class ColorTest extends LinearOpMode {

    public ColorSensor colorSensor;
    @Override
    public void runOpMode() throws InterruptedException {

        colorSensor = hardwareMap.get(ColorSensor.class, "Color");

        while (!isStarted()) {
            telemetry.addData("color values alpha: ", colorSensor.alpha());
            telemetry.addData("color values blue: ", colorSensor.blue());
            telemetry.addData("color values red: ", colorSensor.red());

            telemetry.update();

        }

        waitForStart();

        while (opModeIsActive()) {

        }
    }
}
