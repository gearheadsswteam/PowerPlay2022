package org.firstinspires.ftc.teamcode.robot.mecanum;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.Orientation;
import org.firstinspires.ftc.teamcode.robot.GearheadsMecanumRobotRR;

public class AutonomousMecanumMover {
    //This will need tuning if you see the robot oscillating heavily
    //This is the KP coeefient for Gyro correction to be used during straffing
    public static final double STRAFFING_KP_CORRECTION_FACTOR = 0.1;

    public static final double KP_CORRECTION_FACTOR = 0.1;

    // Set PID proportional value to produce non-zero correction value when robot veers off
    // straight line. P value controls how sensitive the correction is.
    //private PIDController pidDrive = new PIDController(KP_CORRECTION_FACTOR, 0, 0);
    private PIDController pidDrive = new PIDController(.1, 0.09, 0);

    //This is the conversion factor : STRAFE DIST = FORWARD DIST * MECUNNUM_STRAFE_TO_FORWARD_DIST_SCALE
    public static final double MECUNNUM_STRAFE_TO_FORWARD_DIST_SCALE = 1.025;
    /* Declare OpMode members. */
    public MecanumDrive mecanum;
    private BNO055IMU gyro;
    public GearheadsMecanumRobotRR robot = null;   // Use a Gearbot's hardware
    private LinearOpMode curOpMode = null;   //current opmode
    private ElapsedTime runtime = null;//used for measuring time
    private final double FORWARD_SPEED = 0.2;
    public BNO055IMU imu;
    private Orientation lastAngles = new Orientation();
    private double globalAngle;

    boolean isEncoderOn = false; //flag which starts and stops encoder count
    double encoderCount = 0; //counting encoder for parking computation.


    /**
     * Constructor
     *
     * @param gearheadsRobot robot to use
     * @param myOpMode       opmode that is executing
     */
    public AutonomousMecanumMover(GearheadsMecanumRobotRR gearheadsRobot, LinearOpMode myOpMode, MecanumDrive mecanumDrive) {
        robot = gearheadsRobot;
        curOpMode = myOpMode;
        runtime = new ElapsedTime();
        imu = gearheadsRobot.imu;
        mecanum = mecanumDrive;
    }


    /**
     * Moves the robot forward for specific time
     *
     * @param time time to move the robot in milliseconds
     */
    public void moveRobotForwardByTime(double time) {
        moveRobotForwardOrBackwardByTime(time, -FORWARD_SPEED);
    }

    /**
     * Moves the robot forward for specific time
     *
     * @param time  time to move the robot in milliseconds
     * @param speed speed to move in range (0 to 1)
     */
    public void moveRobotForwardByTime(double time, double speed) {
        moveRobotForwardOrBackwardByTime(time, -speed);
    }

    /**
     * Moves the robot backward for specific time
     *
     * @param time time to move the robot in milliseconds
     */
    public void moveRobotBackwardByTime(double time) {
        moveRobotForwardOrBackwardByTime(time, FORWARD_SPEED);
    }

    /**
     * Moves the robot backward for specific time
     *
     * @param time  time to move the robot in milliseconds
     * @param speed speed to move in range (0 to 1)
     */
    public void moveRobotBackwardByTime(double time, double speed) {
        moveRobotForwardOrBackwardByTime(time, speed);
    }


    /**
     * Moves robot forward or backward based on positive or negative speed for fixed time
     *
     * @param time  time to move the robot in milliseconds
     * @param speed speed to move in range (-1 to 1), -ve for forward, +ve for backward
     */
    private void moveRobotForwardOrBackwardByTime(double time, double speed) {
        double x = 0;
        double y = speed;

        mecanum.move(x, y, 0);

        runtime.reset();
        resetAngle();
        pidDrive.initPIDController();

        while (curOpMode.opModeIsActive() && (runtime.milliseconds() < time)) {
            double correction = checkDirection();
            // Compensate for gyro angle.
            Vector2d input = new Vector2d(x, y);
            input.rotate(-correction);

            mecanum.move(input.x, input.y, 0);
        }
        // turn the motors off.
        mecanum.stopRobot();

        pushTelemetry();
        resetAngle();
    }

    /**
     * Push Telemetry data
     */
    private void pushTelemetry() {
        curOpMode.telemetry.addData("Run Time", "Leg 1: %2.5f S Elapsed ", runtime.seconds());
        curOpMode.telemetry.addData("Gyro Heading ", getAngle());
        curOpMode.telemetry.update();
    }

    /**
     * Moves the robot right for specific time
     *
     * @param time time to move the robot in milliseconds
     */
    public void moveRobotRightByTime(double time) {
        moveRobotLeftOrRightByTimeConditionally(time, FORWARD_SPEED);
    }

    /**
     * Moves the robot left or right for specific time or until a condition is met.
     *
     * @param time  time to move the robot in milliseconds
     * @param speed speed to move in range (-1 to 1), -ve for left & +ve for right
     *
     */

    private boolean moveRobotLeftOrRightByTimeConditionally(double time,
                                                            double speed) {
        double x = speed;
        double y = 0;
        boolean conditionMet = false;

        mecanum.move(x, y, 0);

        runtime.reset();
        resetAngle();

        while (curOpMode.opModeIsActive() && (runtime.milliseconds() < time)) {



//            double correction = checkDirection();
//            // Compensate for gyro angle.
//            Vector2d input = new Vector2d(x, y);
//            input.rotate(-correction);
//
//            mecanum.move(input.x, input.y, 0);
        }
        // turn the motors off.
        mecanum.stopRobot();
        this.rotateToZeroHeading(0.1);
        pushTelemetry();

        resetAngle();
        return conditionMet;
    }

    /**
     * Moves the robot right for specific time
     *
     * @param time  time to move the robot in milliseconds
     * @param speed speed to move in range (0 to 1)
     */
    public void moveRobotRightByTime(double time, double speed) {
        moveRobotLeftOrRightByTimeConditionally(time, speed);
    }



    /**
     * Moves the robot left for specific time
     *
     * @param time time to move the robot in milliseconds
     */
    public void moveRobotLeftByTime(double time) {
        moveRobotLeftOrRightByTimeConditionally(time, -FORWARD_SPEED);
    }

    /**
     * Moves the robot left for specific time
     *
     * @param time  time to move the robot in milliseconds
     * @param speed speed to move in range (0 to 1)
     */
    public void moveRobotLeftByTime(double time, double speed) {
        moveRobotLeftOrRightByTimeConditionally(time, -speed);
    }


    /**
     * Rotate right
     *
     * @param degreesToRotate degrees to rotate
     */
    public void rotateRight(int degreesToRotate) {
        rotateRight(degreesToRotate, 0.3);
    }

    /**
     * Rotate left
     *
     * @param degreesToRotate degrees to rotate
     */

    public void rotateLeft(int degreesToRotate) {
        rotateLeft(degreesToRotate, 0.3);
    }

    /**
     * Rotate right
     *
     * @param degreesToRotate degrees to rotate
     * @param power           speed at which to rotate
     */

    public void rotateRight(int degreesToRotate, double power) {
        resetAngle();
        degreesToRotate = Math.abs(degreesToRotate);

        double angleRotated = Math.abs(rotate(Math.abs((int) (degreesToRotate * 0.8)), power));
        curOpMode.sleep(50);
        angleRotated = Math.abs(angleRotated);


        resetAngle();//Very important

        angleRotated += Math.abs(rotate(Math.abs((int) (degreesToRotate - angleRotated)), 0.1));

        curOpMode.telemetry.addData("Needed rotation ", degreesToRotate);
        curOpMode.telemetry.addData("Achieved rotation ", angleRotated);
        curOpMode.telemetry.update();
        resetAngle();
    }


    /**
     * Rotate left
     *
     * @param degreesToRotate degrees to rotate
     * @param power           speed at which to rotate
     */
    public void rotateLeft(int degreesToRotate, double power) {
        resetAngle();
        degreesToRotate = Math.abs(degreesToRotate);

        double angleRotated = Math.abs(rotate(-Math.abs((int) (degreesToRotate * 0.8)), power));
        curOpMode.sleep(50);
        angleRotated = Math.abs(angleRotated);

        resetAngle();//Very important

        angleRotated += Math.abs(rotate(-Math.abs((int) (degreesToRotate - angleRotated)), 0.1));
        curOpMode.telemetry.addData("Needed rotation ", degreesToRotate);
        curOpMode.telemetry.addData("Achieved rotation ", angleRotated);
        curOpMode.telemetry.update();
        resetAngle();
    }

    /**
     * Rotate left or right the number of degrees. Does not support turning more than 180 degrees.
     *
     * @param degrees Degrees to turn, +ve is left -ve is right
     * @param power   speed of movement (Range 0 to 1)
     */
    private double rotate(int degrees, double power) {
        double turn;

        // getAngle() returns + when rotating counter clockwise (left) and - when rotating
        // clockwise (right).

        if (degrees < 0) {   // turn right.
            turn = power;
        } else if (degrees > 0) {   // turn left.
            turn = -power;
        } else return getAngle();

        // set z value to rotate.
        mecanum.move(0, 0, turn);

        // rotate until turn is completed.
        if (degrees < 0) {
            // On right turn we have to get off zero first.
            while (curOpMode.opModeIsActive() && getAngle() == 0) {
                printOrientation();
            }


            while (curOpMode.opModeIsActive() && Math.abs(getAngle()) < Math.abs(degrees)) {
                printOrientation();

            }
        } else    // left turn.

            while (curOpMode.opModeIsActive() && Math.abs(getAngle()) < Math.abs(degrees)) {
                printOrientation();
            }

        // turn the motors off.
        mecanum.stopRobot();

        // wait for rotation to stop.
        curOpMode.sleep(1000);

        return getAngle();
    }

    public void printOrientation() {
        curOpMode.telemetry.addData("Angle ", getAngle());
        curOpMode.telemetry.update();
    }

    /**
     * Rotates till zero heading is achieved
     *
     * @param power speed of rotation. Range (0 to 1)
     */
    private void rotateToZeroHeading(double power) {
        double turn;
        double curHeading = getAngle();

        // getAngle() returns + when rotating counter clockwise (left) and - when rotating
        // clockwise (right).

        if (curHeading < 0) {   // turn left as it is facing right.
            turn = power;
        } else if (curHeading > 0) {   // turn right as it is facing left.
            turn = -power;
        } else return;

        // set power to rotate.
        mecanum.move(0, 0, turn);

        boolean zeroHeadingReached = false;

        // rotate until turn is completed.
        while (curOpMode.opModeIsActive() && !zeroHeadingReached) {
            double heading = getAngle();
            //Stop when heading is within +-0.5 degress, if you set absolute zero the while loop may not complete always
            zeroHeadingReached = (heading < 0.5 && heading > -0.5);
        }

        // turn the motors off.
        mecanum.stopRobot();

        // wait for rotation to stop.
        curOpMode.sleep(1000);
    }

    /**
     * Get current cumulative angle rotation from last reset.
     *
     * @return Angle in degrees. + = left, - = right.
     */
    public double getAngle() {
        // We experimentally determined the Z axis is the axis we want to use for heading angle.
        // We have to process the angle because the imu works in euler angles so the Z axis is
        // returned as 0 to +180 or 0 to -180 rolling back to -179 or +179 when rotation passes
        // 180 degrees. We detect this transition and track the total cumulative angle of rotation.

        Orientation angles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.DEGREES);

        double deltaAngle = angles.firstAngle - lastAngles.firstAngle;

        if (deltaAngle < -180)
            deltaAngle += 360;
        else if (deltaAngle > 180)
            deltaAngle -= 360;

        globalAngle += deltaAngle;

        lastAngles = angles;

        return globalAngle;
    }

    /**
     * Resets the cumulative angle tracking to zero.
     */
    private void resetAngle() {
        lastAngles = imu.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX,
                AngleUnit.DEGREES);

        globalAngle = 0;
    }

    private double checkDirection() {
        return checkDirection(KP_CORRECTION_FACTOR);
    }

    /**
     * See if we are moving in a straight line and if not return a power correction value.
     *
     * @return Power adjustment, + is adjust left - is adjust right.
     */
    private double checkDirection(double neededGain) {
        // The gain value determines how sensitive the correction is to direction changes.
        // You will have to experiment with your robot to get small smooth direction changes
        // to stay on a straight line.
        double correction, angle, gain = .10;

        gain = neededGain;

        angle = getAngle();

        curOpMode.telemetry.addData("II Gyro ", angle);

        if (angle == 0)
            correction = 0;             // no adjustment.
        else
            correction = pidDrive.performPID(angle);
        ;        // reverse sign of angle for correction.

        //correction = correction * gain;

        return correction;
    }

    /**
     * See if we are moving in a straight line and if not return a power correction value.
     *
     * @return Power adjustment, + is adjust left - is adjust right.
     */
    private double checkDirectionStrafe(double neededGain) {
        // The gain value determines how sensitive the correction is to direction changes.
        // You will have to experiment with your robot to get small smooth direction changes
        // to stay on a straight line.
        double correction, angle, gain = .10;

        gain = neededGain;

        angle = getAngle();

        curOpMode.telemetry.addData("II Gyro ", angle);

        if (angle == 0)
            correction = 0;             // no adjustment.
        else
            correction = -angle;        // reverse sign of angle for correction.

        correction = correction * gain;

        return correction;
    }




    /**
     * Makes corections for robot heading using Gyro during strafing
     *
     * @param x
     * @param y
     */
    private void makeGyroCorrectionsDuringStrafing(double x, double y) {
        double correctionFactor = STRAFFING_KP_CORRECTION_FACTOR; //Tune this for your robot to prevent wobbling
        double directionError = checkDirectionStrafe(KP_CORRECTION_FACTOR);
        //TODO - we can use a PID controller class to improve the error correction if just proportional methos does not work
        double correctionApply = -correctionFactor * directionError;
        mecanum.move(x, y, correctionApply);
        curOpMode.sleep(200);
    }

    /*
     *  Method to perfmorm a relative move, based on encoder counts.
     *  Encoders are not reset as the move is based on the current position.
     *  Move will stop if any of three conditions occur:
     *  1) Move gets to the desired position
     *  2) Move runs out of time
     *  3) Driver stops the opmode running.
     *  4) Search condition is met
     */


    /**
     * Starts Encoder counting for parking computation
     */

    public void startEncoderCounting() {
        isEncoderOn = true;
    }

    /**
     * Stops Encoder counting for parking computation
     */
    public void stopEncoderCounting() {
        isEncoderOn = false;
    }


    /**
     * resets  Encoder count used in parking computation
     */
    public void resetEncoderCounting() {
        encoderCount = 0;
    }
}