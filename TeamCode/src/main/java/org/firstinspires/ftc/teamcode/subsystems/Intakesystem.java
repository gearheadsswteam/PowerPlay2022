package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.robot.ValueStorage;

/**
 * Class the represents the Intake System responsble for taking in the Cargo
 */
public class Intakesystem {
    //DC motor used by the intake system
    public DcMotor intakeMotorL; // intake motor

    //DC motor used by the intake system
    public DcMotor intakeMotorR; //Roller motor

    public Servo rollerServo;


    /**
     * Constructor
     * @param intakeMotor intake motor
     * @param stackMotor intake gate servo
     */
    public Intakesystem(DcMotor intakeMotorL, DcMotor intakeMotorR, Servo rolerServo) {
        this.intakeMotorL = intakeMotorL;
        this.intakeMotorR = intakeMotorR;
        this.rollerServo = rolerServo;
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
        intakeMotorR.setPower(0.5);
        intakeMotorL.setPower(0.25);

    }

    /**
     * Start the intake system
     */
    public void reverseInTake() {
        intakeMotorR.setPower(-0.5);
        intakeMotorL.setPower(-0.5);
    }


    /**
     * Stop the intake system
     */
    public void stopInTake() {
        intakeMotorR.setPower(0);
        intakeMotorL.setPower(0);
    }

    public void setRollerUp(){
        rollerServo.setPosition(ValueStorage.ROLLER_UP);
    }

    public void setRollerDown(){
        rollerServo.setPosition(ValueStorage.ROLLER_DOWN);
    }
}
