package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class ConeDeliverySystem {
    private DcMotor elevatorL;
    private DcMotor elevatorR;

    private Servo armL;
    private Servo armR;

    private Servo wristL;
    private Servo wristR;

    private Servo gripper;

    public ConeDeliverySystem(HardwareMap hwMap){
        elevatorL = hwMap.dcMotor.get("x");
        elevatorR = hwMap.dcMotor.get("x");

        armL = hwMap.servo.get("x");
        armR = hwMap.servo.get("x");

        wristL = hwMap.servo.get("x");
        wristR = hwMap.servo.get("x");

        gripper = hwMap.servo.get("gripper");
    }

    public void initialize(){
        //This is based on how motors have been mounted
        elevatorL.setDirection(DcMotor.Direction.FORWARD);
        elevatorR.setDirection(DcMotor.Direction.FORWARD);

        elevatorL.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        elevatorR.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }


    public void setPosition(double [] elevatorPostion){
        elevatorL.setTargetPosition((int)elevatorPostion[0]);
        elevatorR.setTargetPosition(-(int)elevatorPostion[0]);

        armL.setPosition(elevatorPostion[1]);
        armR.setPosition(1-elevatorPostion[1]);

        wristL.setPosition(elevatorPostion[2]);
        wristR.setPosition(1-elevatorPostion[2]);
    }
}
