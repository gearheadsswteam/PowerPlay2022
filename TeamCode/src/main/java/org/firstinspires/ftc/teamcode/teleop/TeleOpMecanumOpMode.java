package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;

import org.firstinspires.ftc.teamcode.robot.GearheadsMecanumRobotRR;
import org.firstinspires.ftc.teamcode.robot.mecanum.MecanumDrive;
import org.firstinspires.ftc.teamcode.robot.ValueStorage;
import org.firstinspires.ftc.teamcode.subsystems.ConeDeliverySystem;
import org.firstinspires.ftc.teamcode.subsystems.Intakesystem;


@TeleOp(name = "TeleOpTwoDriver", group = "TeleOp")
//@Disabled
public class TeleOpMecanumOpMode extends LinearOpMode {

    //Reference for Josh's code: https://docs.google.com/document/d/1nJ-Rro6GFyXt1vbN69c-Y5u8U8c_oHpr-_ET3eomAbA/edit

    /* Declare OpMode members. */
    private GearheadsMecanumRobotRR robot;   // Use gearheads robot hardware
    private Intakesystem intakesystem;
    private ConeDeliverySystem coneDeliverySystem;


    private MecanumDrive mecanum;
    private BNO055IMU gyro;

    private double turn;
    private double forwardPower;
    private double sidePower;

    private boolean capstoneArmTriggerUp = true;
    private int capstoneArmState = 0;


    /**
     * Constructor
     */
    public TeleOpMecanumOpMode() {

        robot = new GearheadsMecanumRobotRR(this);
    }

    @Override
    public void runOpMode() throws InterruptedException {
        // Wait for the game to start (driver presses PLAY)
        //Need this as first step else we get 5 point penalty
        waitForStart();

        initOpMode();

        while (opModeIsActive()) {
            adjustForFOV();

            dampenSpeed();
            //Move The robot
            moveRobot();

            operateRoller();
        }
    }

    public void operateRoller(){
        intakesystem.setRollerUp();
        intakesystem.startInTake();
    }

    /**
     * Initialize the opmode
     */

    private void initOpMode() {
        // Wait for the game to start (driver presses PLAY)
        //Need this as first step else we get 5 point penalty
        waitForStart();

        telemetry.addData("Status", "Started");

        telemetry.update();

        /*
         * Initialize the drive system variables.
         * The init() method of the hardware class does all the work here
         */
        robot.initTeleOp(hardwareMap);
        intakesystem = robot.intakesystem;
        coneDeliverySystem = robot.coneDeliverySystem;



        gyro = robot.imu;

        telemetry.addData("Status", "Initialized");
        telemetry.update();

        //DRIVING
        DcMotor fl_motor = robot.fl_motor;
        DcMotor fr_motor = robot.fr_motor;
        DcMotor rl_motor = robot.rl_motor;
        DcMotor rr_motor = robot.rr_motor;

        mecanum = new MecanumDrive(fl_motor, fr_motor, rl_motor, rr_motor, gyro);
    }


    /**
     * Adjust teleop driving with FOV mode
     */
    private void adjustForFOV() {

        //Angle adjustment during TeleOP based on how the autonomous ends. In Game changers the TeleOPs starts with Rbot at 90 degrees from FOV.
        double angleFromAutonomousLastRun = ValueStorage.lastPose.getHeading();
        //double angleFromAutonomousLastRun = Math.PI/2;

        double angle = 0;
        if (ValueStorage.redMultiplier == 1) {//Red team
            angle = gyro.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS).firstAngle + angleFromAutonomousLastRun - Math.PI / 2;
        } else {
            angle = gyro.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS).firstAngle + angleFromAutonomousLastRun + Math.PI / 2;
        }


        //Read  joystick values without FOV compensation
        double tempForwardPower = gamepad1.left_stick_y;
        double tempSidePower = gamepad1.left_stick_x;

        //Adjust X & Y power based on FOV
        sidePower = tempForwardPower * Math.cos(angle) + tempSidePower * Math.sin(angle);
        forwardPower =-tempForwardPower * Math.sin(angle) + tempSidePower * Math.cos(angle);

        //Read turn commands
        turn = gamepad1.right_stick_x;
    }

    /**
     * Dampen the Robot driving movements if right trigger is pressed
     */
    private void dampenSpeed() {
        float speedDamper = gamepad1.right_trigger;

        if (speedDamper == 1) {
            speedDamper = (float) 0.8;
        }

        forwardPower = forwardPower * (1 - speedDamper);
        sidePower = sidePower * (1 - speedDamper);
        turn = turn * (1 - speedDamper);
    }



    /**
     * Drive the robot
     */
    private void moveRobot() {
        //Joystick Movement
        mecanum.move(sidePower, forwardPower, turn);
        //Push data
        pushTelemetry();
    }

    /**
     * Drive the robot
     */
    private void stopRobot() {
        //Joystick Movement
        mecanum.move(0, 0, 0);
        //Push data
        pushTelemetry();
    }


    private void pushTelemetry() {
        telemetry.addData("Gyro Heading", gyro.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS).firstAngle);
        telemetry.addData("Drive Data", mecanum.getDataString());
        telemetry.update();
    }

}