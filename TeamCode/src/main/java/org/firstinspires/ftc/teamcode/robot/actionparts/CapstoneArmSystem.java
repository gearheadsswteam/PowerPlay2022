package org.firstinspires.ftc.teamcode.robot.actionparts;

import com.qualcomm.robotcore.hardware.Servo;

/**
 * Capstone Arm subsystem used to pick up Game element and deliver
 */
public class CapstoneArmSystem {

    //The Servo to lift the arm up and down
    private Servo liftServo;

    //Servo used to grab the capstone
    private Servo grabServo;

    //Arm Positions
    public static final double ARM_REST = 0.43;
    private final double ARM_UP = 0.65;
    private final double ARM_DOWN = 0.795;


    //Claw position
    private final double CLOSED_POSITION = 0.76;
    private final double OPEN_POSITION = 0.43;

    private boolean isOpen = true;

    /**
     * Constructor
     *
     * @param liftServo
     * @param grabServo
     */
    public CapstoneArmSystem(Servo liftServo, Servo grabServo) {
        this.liftServo = liftServo;
        this.grabServo = grabServo;
    }

    /**
     * Initialize the Capstone System
     */
    public void initialize() {
        grabCapstone();
        armUp();
    }

    /**
     * Grabs the wobble goal
     */
    public void grabCapstone() {
        grabServo.setPosition(CLOSED_POSITION);
        isOpen = false;
    }

    /**
     * Ungrabs the wobble goal
     */
    public void ungrabCapstone() {
        grabServo.setPosition(OPEN_POSITION);
        isOpen = true;
    }

    /**
     * Lifts the wobble goal post
     */

    public void armUp() {
        liftServo.setPosition(ARM_REST);
    }

    /**
     * Sets the wobble goal post down
     */
    public void armDown() {
        liftServo.setPosition(ARM_DOWN);
    }


    /**
     * Sets the wobble goal post down
     */
    public void armRest() {
        liftServo.setPosition(ARM_REST);
    }

    /**
     * Method is a state machine which move the Capstone arm postion and grip based on one button on the driver side
     *
     * @param capstonearmState the state for the capstone arm
     */
    public void moveCapstoneArm(int capstonearmState) {

        switch (capstonearmState) {
            case 0:
                ungrabCapstone();
                armUp();
                break;
            case 1:
                ungrabCapstone();
                armDown();
                break;
            case 2:
                grabCapstone();
                armDown();
                break;
            case 3:
                grabCapstone();
                armUp();
                break;
        }
    }

    /**
     * Sets arm position to a specific place
     * @param postionToSet encoder value for position
     */
    public void setArmPositon(double postionToSet) {
        if (postionToSet < ARM_DOWN && postionToSet > ARM_REST) {
            liftServo.setPosition(postionToSet);
        }
    }

    /**
     * Lowers the arm
     */
    public void lowerArm() {
        double curPositon = liftServo.getPosition();
        double newPositon = curPositon - 0.05;
        if (newPositon > this.ARM_REST) {
            liftServo.setPosition(newPositon);
        }
    }

    /**
     * Toggles the claw open close
     */
    public void toogleGrip() {
        if (isOpen) {
            grabServo.setPosition(CLOSED_POSITION);
            isOpen = false;
        }
        if (!isOpen) {
            grabServo.setPosition(OPEN_POSITION);
            isOpen = true;
        }
    }
}
