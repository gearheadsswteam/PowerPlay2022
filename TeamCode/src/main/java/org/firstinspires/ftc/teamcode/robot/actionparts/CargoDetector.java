package org.firstinspires.ftc.teamcode.robot.actionparts;

import com.qualcomm.robotcore.hardware.DistanceSensor;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

/**
 * Detects if the cargo bucket is full
 */
public class CargoDetector {
    //The distance sensor used for detection
    private DistanceSensor disSensor;

    //Distance indicating cargo
    private double EMPTY_CARGO_DIST = 50; //Empty distance is 57 mm, anything less means bucket has something


    /**
     * Constructor
     * @param distanceSensor The distance sensor used for detection
     */
    public CargoDetector(DistanceSensor distanceSensor) {
        this.disSensor = distanceSensor;
    }

    /**
     * Checks if the bucket is full
     * @return true if bucket is full
     */
    public boolean isCargoBucketFull() {
        if (disSensor.getDistance(DistanceUnit.MM) < EMPTY_CARGO_DIST) {
            return true;
        } else {
            return false;
        }
    }
}
