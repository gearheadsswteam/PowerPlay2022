package org.firstinspires.ftc.teamcode.autonomousRR;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.teamcode.drive.PoseStorage;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.robot.GearheadsMecanumRobotRR;
import org.firstinspires.ftc.teamcode.robot.drivetrain.mecanum.AutonomousMecanumMoverRR;


@Autonomous(name = "Mecannum: AbstractAutonomousOpModeRR", group = "Mecannum")
@Disabled
/*
  This is the baseclass for all autonomous op modes for Meccunum robot
 */
public abstract class AbstractAutonomousOpModeRR extends LinearOpMode {
    //Red Team type
    public static final String RED_TEAM = "redteam";

    //Blue Team type
    public static final String BLUE_TEAM = "blueteam";

    //Team type
    protected String TEAM_TYPE;

    // Use gearheads robot hardware
    public GearheadsMecanumRobotRR robot;
    //The drive system
    public SampleMecanumDrive mecanumDriveRR;

    // The autonomous driving software
    protected AutonomousMecanumMoverRR autonomousRobotMover;


    /**
     * Constructor
     */
    public AbstractAutonomousOpModeRR() {
        robot = new GearheadsMecanumRobotRR(this);
    }

    /**
     * Run this as the first thing in the autonomous opmode
     */

    protected void initOpModeBeforeStart() {
        /*
         * Initialize the drive system variables.
         * The init() method of the hardware class does all the work here
         */
        robot.initAutonomous(hardwareMap, TEAM_TYPE);


        mecanumDriveRR = new SampleMecanumDrive(hardwareMap);
        autonomousRobotMover = new AutonomousMecanumMoverRR(robot, this, mecanumDriveRR);

        telemetry.addData("Status", "Initialized " + TEAM_TYPE);
        telemetry.update();
    }


    /**
     * Run this as the first thing in the autonomous opmode after initialization
     */
    protected abstract void initOpModeAfterStart();


    /**
     * This is where the actual opmode logic should be implemented
     */
    protected abstract void executeOpMode();

    @Override
    public void runOpMode() {
        initOpModeBeforeStart();
        waitForDriverAcknowledgement();
        initOpModeAfterStart();

        // Wait for the game to start (driver presses PLAY)
        while (opModeIsActive()) {
            executeOpMode();
            break;
        }

        closeOpMode();
    }

    /**
     * This method pause the op mode till driver presses start
     */
    private void waitForDriverAcknowledgement() {
     //Add code here

        // Wait for the game to start (driver presses PLAY)
        waitForStart();
    }

    /**
     * This method pause the op mode till driver presses start
     */
    private void closeOpMode() {

        // Transfer the current pose to PoseStorage so we can use it in TeleOp
        PoseStorage.currentPose = mecanumDriveRR.getPoseEstimate();
        PoseStorage.gyroAngle= robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS).firstAngle;
        PoseStorage.TEAM_TYPE = this.TEAM_TYPE;

        // Prompt User
        telemetry.addData(">", "OpMode complete " + this.getClass().getSimpleName());
        telemetry.addData("End State Team | gyro angle ",  PoseStorage.gyroAngle + " | " + PoseStorage.TEAM_TYPE);
        telemetry.update();
    }

    /**
     * Turns robot left or right for power shots
     */
    private void setZeroHeading() {
            double angle = robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS).firstAngle ;
            double headingCorrection = -angle;
            mecanumDriveRR.turn(headingCorrection);
    }

    /**
     * Turns robot left or right for power shots
     */
    private void printHeading() {
        double gyroHeading = robot.imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS).firstAngle ;
        double odometryHeading = mecanumDriveRR.getPoseEstimate().getHeading();

        // Prompt User
        telemetry.addData("gyroHeading ", Math.toDegrees(gyroHeading));
        telemetry.addData("odometryHeading ", Math.toDegrees(odometryHeading));
        telemetry.update();
    }
}


