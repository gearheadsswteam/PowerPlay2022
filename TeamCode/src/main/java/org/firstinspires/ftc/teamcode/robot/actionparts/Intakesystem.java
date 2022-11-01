package org.firstinspires.ftc.teamcode.robot.actionparts;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

/**
 * Class the represents the Intake System responsble for taking in the Cargo
 */
public class Intakesystem {
    //DC motor used by the intake system
    public DcMotor intakeMotor;

    //DC motor used by the intake system
    public DcMotor stackMotor;


    /**
     * Constructor
     * @param intakeMotor intake motor
     * @param stackMotor intake gate servo
     */
    public Intakesystem(DcMotor intakeMotor, DcMotor stackMotor) {
        this.intakeMotor = intakeMotor;
        this.stackMotor = stackMotor;
    }



    /**
     * Initialize the system
     */
    public void initialize() {

    }

    /**
     * Start the intake system
     */
    public void startInTake() {
        intakeMotor.setPower(-0.5);

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
    public void startStackMotor() {
        stackMotor.setPower(-0.5);

    }

    /**
     * Stop the intake system
     */
    public void stopStackMotor() {
        stackMotor.setPower(0);
    }


}
