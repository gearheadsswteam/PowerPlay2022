package org.firstinspires.ftc.teamcode.robot.actionparts;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * Class the represents the Intake System responsble for taking in the Cargo
 */
public class Intakesystem {
    //DC motor used by the intake system
    public DcMotor intakeMotor;

    //Servo for the intake gate used to ensure only one cargo is taken in at a time
    public Servo intakeServo;

    //Intake gate up position
    private double gateUp = 0.65;

    //Intake gate down position
    private double gateDown = 0.31;

    //State of the intake gate
    private boolean intakeOpen = true;

    /**
     * Constructor
     * @param intakeMotor intake motor
     * @param intakeServo intake gate servo
     */
    public Intakesystem(DcMotor intakeMotor, Servo intakeServo) {
        this.intakeMotor = intakeMotor;
        this.intakeServo = intakeServo;
    }

    /**
     * Toggles the state of the intake gate
     */
    public void toggleIntakeOpenClosePosition() {
        if (intakeOpen) {
            closeIntakeGate();
        } else {
            openIntakeGate();
        }
    }

    /**
     * opens the intake gate
     */

    public void openIntakeGate() {
        intakeServo.setPosition(gateDown);
        intakeOpen = true;
    }

    /**
     * Closes the intake gate
     */
    public void closeIntakeGate() {
        intakeServo.setPosition(gateUp);
        intakeOpen = false;
    }

    /**
     * Initialize the system
     */
    public void initialize() {
        intakeServo.setPosition(gateDown);
        intakeOpen = false;
    }

    /**
     * Start the intake system
     */
    public void startInTake() {
        intakeMotor.setPower(-0.5);
        // intakeServo.setPosition(servoIntakeBlockPosition);
    }

    /**
     * Stop the intake system
     */
    public void stopInTake() {
        intakeMotor.setPower(0);
    }

    /**
     * Start the intake system
     */
    public void startReverseInTake() {
        intakeMotor.setPower(1.0);
    }
}
