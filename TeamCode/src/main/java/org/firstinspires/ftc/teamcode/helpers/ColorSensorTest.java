package org.firstinspires.ftc.teamcode.helpers;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@Autonomous(name = "Color Sensor Test", group = "Test")
@Disabled
public class ColorSensorTest extends LinearOpMode {
    /* local OpMode members. */
    ColorSensor colorSensor = null;
    DistanceSensor sensorDistance;

    public ColorSensorTest() {

    }


    @Override
    public void runOpMode() throws InterruptedException {
        /*
         * Initialize the drive system variables.
         * The init() method of the hardware class does all the work here
         */
        // Connect to servo (Assume PushBot Left Hand)
        // Change the text in quotes to match any servo name on your robot.
        colorSensor = hardwareMap.get(ColorSensor.class, "colorSensor");
        // get a reference to the distance sensor that shares the same name.
        sensorDistance = hardwareMap.get(DistanceSensor.class, "colorSensor");

        // Set the LED in the beginning
        colorSensor.enableLed(false);


        // Send telemetry message to signify robot waiting;
        telemetry.addData("Status", "Ready to See"); //
        telemetry.update();

        waitForStart();
        // while the op mode is active, loop and read the RGB data.
        // Note we use opModeIsActive() as our loop condition because it is an interruptible method.
        while (opModeIsActive()) {
            // send the info back to driver station using telemetry function.
            //telemetry.addData("LED", robot.bLedOn ? "On" : "Off");

            telemetry.addData("Clear", colorSensor.alpha());
            telemetry.addData("Red ", colorSensor.red());
            telemetry.addData("Green", colorSensor.green());
            telemetry.addData("Blue ", colorSensor.blue());
            telemetry.addData("Dist ", sensorDistance.getDistance(DistanceUnit.MM));
//            telemetry.addData("CC ", getCCValue(colorSensor));

            telemetry.update();
            idle();
        }
    }


    private boolean isSkystone(ColorSensor colorSensor) {
        double cc = colorSensor.red() * colorSensor.green() / colorSensor.blue();
        return cc < 3;
    }

    private double getCCValue(ColorSensor colorSensor) {
        return colorSensor.red() * colorSensor.green() / colorSensor.blue();
    }


}