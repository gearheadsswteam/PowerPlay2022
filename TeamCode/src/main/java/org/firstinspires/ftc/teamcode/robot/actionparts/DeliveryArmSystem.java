package org.firstinspires.ftc.teamcode.robot.actionparts;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * Delivery arm system responsible for deliverying the cargo to the shipping hubs
 */
public class DeliveryArmSystem {

    //The Motor to lift the Elevator
    private DcMotor liftElevator;

    //Servo to tilt the bucket
    private Servo tiltBucket;

    //State of the tilt bucket
    private boolean isTilted = false;

    //Bucket servo Positions
    private final double BUCKET_REST = 0.36;
    private final double BUCKET_UP = 0.58;
    private final double BUCKET_DOWN = 0.80;

    //Elevator encoder position values
    private final int ELEVATOR_POSITION_LOW = 0;
    private final int ELEVATOR_POSITION_MED = -360;
    private final int ELEVATOR_POSITION_HIGH = -940;

    //Opmode reference
    private LinearOpMode curOpMode;

    //State of the bucket
    boolean bucketMoveComplete = true;


    /**
     *
      * @param liftElevator
     * @param tiltBucket
     * @param curOpMode
     */
    public DeliveryArmSystem(DcMotor liftElevator, Servo tiltBucket, LinearOpMode curOpMode) {
        this.liftElevator = liftElevator;
        this.tiltBucket = tiltBucket;
        this.curOpMode = curOpMode;
    }

    /**
     * Initialize the Delivery Arm System
     */

    public void initialize() {
        liftElevator.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        liftElevator.setPower(1);
        setLiftElevatorLow();
    }

    /**
     * Sets elevator to lowest position
     */
    public void setLiftElevatorLow() {
        setElevatorHeight(this.ELEVATOR_POSITION_LOW);
    }

    /**
     * Sets elevator to medium position
     */
    public void setLiftElevatorMedium() {
        setElevatorHeight(this.ELEVATOR_POSITION_MED);
    }

    /**
     * Sets elevator to high position
     */
    public void setLiftElevatorHigh() {
        setElevatorHeight(this.ELEVATOR_POSITION_HIGH);
    }

    /**
     * setting the delivery arms position
     */
    private void setElevatorHeight(int elevatorHeight) {
        liftElevator.setTargetPosition(elevatorHeight);
        // Turn On RUN_TO_POSITION
        liftElevator.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        liftElevator.setPower(Math.abs(1));
    }

    /**
     * Toggles the bucket between rest and down position
     */
    public void moveBucket() {
        if (bucketMoveComplete) {
            bucketMoveComplete = false;
            if (isTilted) {
                bucketRest();
                isTilted =false;
            } else {
                bucketDown();
                isTilted = true;
            }
            bucketMoveComplete = true;
        }
    }

    /**
     * Sets bucket to down position
     */
    public void bucketDown() {
        tiltBucket.setPosition(BUCKET_DOWN);
        curOpMode.sleep(700);
    }

    /**
     * Sets bucket to up position ..intermediate position used in Autonomous
      */
    public void bucketUp() {
        tiltBucket.setPosition(BUCKET_UP);
        curOpMode.sleep(700);
    }

    /**
     * Sets bucket to rest position
     */
    public void bucketRest() {
        tiltBucket.setPosition(BUCKET_REST);
        curOpMode.sleep(700);
    }
}
