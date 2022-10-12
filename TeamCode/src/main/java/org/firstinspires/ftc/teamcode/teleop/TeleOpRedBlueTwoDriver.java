package org.firstinspires.ftc.teamcode.teleop;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor.RunMode;
import com.qualcomm.robotcore.hardware.DcMotor.ZeroPowerBehavior;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple.Direction;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.teamcode.ValueStorage;

import static java.lang.Math.PI;
import static java.lang.Math.atan2;
import static java.lang.Math.pow;
import static java.lang.Math.sin;
import static org.firstinspires.ftc.teamcode.ValueStorage.armDown;
import static org.firstinspires.ftc.teamcode.ValueStorage.armRest;
import static org.firstinspires.ftc.teamcode.ValueStorage.armStateTimes;
import static org.firstinspires.ftc.teamcode.ValueStorage.armUp;
import static org.firstinspires.ftc.teamcode.ValueStorage.bucketDetectionFrames;
import static org.firstinspires.ftc.teamcode.ValueStorage.bucketDown;
import static org.firstinspires.ftc.teamcode.ValueStorage.bucketRest;
import static org.firstinspires.ftc.teamcode.ValueStorage.bucketSensorThreshold;
import static org.firstinspires.ftc.teamcode.ValueStorage.bucketUp;
import static org.firstinspires.ftc.teamcode.ValueStorage.clawClosed;
import static org.firstinspires.ftc.teamcode.ValueStorage.clawOpen;
import static org.firstinspires.ftc.teamcode.ValueStorage.gateDown;
import static org.firstinspires.ftc.teamcode.ValueStorage.gateUp;
import static org.firstinspires.ftc.teamcode.ValueStorage.intakeStateTimes;
import static org.firstinspires.ftc.teamcode.ValueStorage.lastArmState;
import static org.firstinspires.ftc.teamcode.ValueStorage.lastIntakeState;
import static org.firstinspires.ftc.teamcode.ValueStorage.lastLiftState;
import static org.firstinspires.ftc.teamcode.ValueStorage.liftPositions;
import static org.firstinspires.ftc.teamcode.ValueStorage.redMultiplier;

@TeleOp(name = "TeleOpTwoController", group = "TeleOp")
public class TeleOpRedBlueTwoDriver extends LinearOpMode {
    DcMotorEx fr;
    DcMotorEx fl;
    DcMotorEx br;
    DcMotorEx bl;
    DcMotorEx intake;
    DcMotorEx lift;
    Servo arm;
    Servo claw;
    Servo bucket;
    Servo gate;
    CRServo spinner;
    BNO055IMU gyro;
    RevColorSensorV3 bucketSensor;
    double initialHeading = ValueStorage.lastPose.getHeading() - redMultiplier * PI / 2;
    double robotHeading;
    double joystickAngle;
    double joystickMagnitude;
    double turn;
    double armUpPos = armUp;
    int intakeState = lastIntakeState;
    int liftState = lastLiftState;
    int armState = lastArmState;
    int detectionFrames = 0;
    ElapsedTime intakeStateTime = new ElapsedTime();
    ElapsedTime armStateTime = new ElapsedTime();
    boolean aPressed = false;
    boolean bPressed = false;
    boolean yPressed = false;
    boolean lbPressed = false;
    boolean aReleased = true;
    boolean bReleased = true;
    boolean yReleased = true;
    boolean lbReleased = true;
    boolean xPressed = false;
    boolean xReleased = true;
    @Override
    public void runOpMode() {
        fr = hardwareMap.get(DcMotorEx.class, "fr");
        fl = hardwareMap.get(DcMotorEx.class, "fl");
        br = hardwareMap.get(DcMotorEx.class, "br");
        bl = hardwareMap.get(DcMotorEx.class, "bl");
        intake = hardwareMap.get(DcMotorEx.class, "intake");
        lift = hardwareMap.get(DcMotorEx.class, "lift");
        arm = hardwareMap.get(Servo.class, "arm");
        claw = hardwareMap.get(Servo.class, "claw");
        bucket = hardwareMap.get(Servo.class, "bucket");
        gate = hardwareMap.get(Servo.class, "gate");
        spinner = hardwareMap.get(CRServo.class, "spinner");
        gyro = hardwareMap.get(BNO055IMU.class, "gyro");
        bucketSensor = hardwareMap.get(RevColorSensorV3.class, "bucketSensor");
        fr.setZeroPowerBehavior(ZeroPowerBehavior.BRAKE);
        fl.setZeroPowerBehavior(ZeroPowerBehavior.BRAKE);
        br.setZeroPowerBehavior(ZeroPowerBehavior.BRAKE);
        bl.setZeroPowerBehavior(ZeroPowerBehavior.BRAKE);
        fr.setDirection(Direction.REVERSE);
        br.setDirection(Direction.REVERSE);
        intake.setDirection(Direction.REVERSE);
        spinner.setDirection(Direction.REVERSE);
        lift.setTargetPosition(liftPositions[liftState]);
        lift.setMode(RunMode.RUN_TO_POSITION);
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();
        parameters.mode = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit = BNO055IMU.AngleUnit.RADIANS;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        gyro.initialize(parameters);
        waitForStart();

        //Stop motors to prevent accidental starts after autonomous
        fr.setPower(0);
        fl.setPower(0);
        br.setPower(0);
        bl.setPower(0);

        lift.setPower(1);
        intakeStateTime.reset();
        armStateTime.reset();
        while (opModeIsActive() && !isStopRequested()) {
            if (gamepad2.a) {
                aPressed = aReleased;
                aReleased = false;
            } else {
                aPressed = false;
                aReleased = true;
            }
            if (gamepad2.b) {
                bPressed = bReleased;
                bReleased = false;
            } else {
                bPressed = false;
                bReleased = true;
            }
            if (gamepad2.y) {
                yPressed = yReleased;
                yReleased = false;
            } else {
                yPressed = false;
                yReleased = true;
            }
            if (gamepad2.x) {
                xPressed = xReleased;
                xReleased = false;
            } else {
                xPressed = false;
                xReleased = true;
            }
            if (gamepad1.left_bumper) {
                lbPressed = lbReleased;
                lbReleased = false;
            } else {
                lbPressed = false;
                lbReleased = true;
            }
            if (gamepad1.ps) {
                initialHeading -= robotHeading;
            }
            if (gamepad1.dpad_up) {
                armUpPos += 0.001;
            } else if (gamepad1.dpad_down) {
                armUpPos -= 0.001;
            }
            if (armUpPos > 0.34) {
                armUpPos = 0.34;
            } else if (armUpPos < 0.24) {
                armUpPos = 0.24;
            }
            if (gamepad2.right_bumper) {
                spinner.setPower(redMultiplier);
            } else if(gamepad2.left_trigger > 0.1){// This is in case the spinner rotation fails due to tele reset post autonomous
                spinner.setPower(-redMultiplier);
            }else {
                spinner.setPower(0);
            }
            if (bucketSensor.getDistance(DistanceUnit.CM) < bucketSensorThreshold) {
                detectionFrames++;
            } else {
                detectionFrames = 0;
            }


            switch (intakeState) {
                case 0:
                    if (intakeStateTime.milliseconds() < intakeStateTimes[0][liftState]) {
                        intake.setPower(0);
                        gate.setPosition(gateUp);
                        bucket.setPosition(bucketRest);
                        if (liftState == 0 || intakeStateTime.milliseconds() >= intakeStateTimes[0][0]) {
                            lift.setTargetPosition(liftPositions[3]);
                        }
                    } else {
                        intake.setPower(1);
                        if (aPressed || bPressed || yPressed || xPressed) {
                            if (aPressed) {
                                liftState = 0;
                            } else if (bPressed) {
                                liftState = 1;
                            } else if (yPressed) {
                                liftState = 2;
                            } else {
                                liftState = 4;
                            }
                            intakeState = 1;
                            intakeStateTime.reset();
                        }
                    }
                    break;
                case 1:
                    if (intakeStateTime.milliseconds() < intakeStateTimes[1][liftState]) {
                        intake.setPower(1);
                        gate.setPosition(gateDown);
                    } else if (detectionFrames >= bucketDetectionFrames || aPressed) {
                        intakeState = 2;
                        intakeStateTime.reset();
                    }
                    break;
                case 2:
                    if (intakeStateTime.milliseconds() < intakeStateTimes[2][liftState]) {
                        intake.setPower(-0.5);
                        bucket.setPosition(bucketUp);
                        lift.setTargetPosition(liftPositions[liftState]);
                    } else {
                        if (bPressed) {
                            intakeState = 3;
                            intakeStateTime.reset();
                        }
                    }
                    break;
                case 3:
                    if (intakeStateTime.milliseconds() < intakeStateTimes[3][liftState]) {
                        bucket.setPosition(bucketDown);
                    } else if (bPressed) {
                        intakeState = 0;
                        intakeStateTime.reset();
                    }
                    break;
            }
            switch (armState) {
                case 0: //claw closed, arm in rest position
                    if (armStateTime.milliseconds() < armStateTimes[0]) {
                        arm.setPosition(armRest);
                        claw.setPosition(clawClosed);
                    } else if (lbPressed) {
                        armState = 1;
                        armStateTime.reset();
                        armUpPos = armUp;
                    }
                    break;
                case 1: //claw closed, arm in drop position
                    arm.setPosition(armUpPos);
                    if (armStateTime.milliseconds() < armStateTimes[1]) {
                    } else if (lbPressed) {
                        armState = 2;
                        armStateTime.reset();
                    }
                    break;
                case 2: //claw open, arm in drop position
                    if (armStateTime.milliseconds() < armStateTimes[2]) {
                        claw.setPosition(clawOpen);
                    } else if (lbPressed) {
                        armState = 3;
                        armStateTime.reset();
                    }
                    break;
                case 3: //claw open, arm in rest position
                    if (armStateTime.milliseconds() < armStateTimes[3]) {
                        arm.setPosition(armRest);
                        claw.setPosition(clawOpen);
                    } else if (lbPressed) {
                        armState = 4;
                        armStateTime.reset();
                    }
                    break;
                case 4: //claw open, arm in grab position
                    if (armStateTime.milliseconds() < armStateTimes[4]) {
                        arm.setPosition(armDown);
                    } else if (lbPressed) {
                        armState = 5;
                        armStateTime.reset();
                    }
                    break;
                case 5: //claw closed, arm in grab position
                    if (armStateTime.milliseconds() < armStateTimes[5]) {
                        claw.setPosition(clawClosed);
                    } else if (lbPressed) {
                        armState = 0;
                        armStateTime.reset();
                    }
                    break;
            }
            robotHeading = gyro.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS).firstAngle + initialHeading;
            joystickAngle = atan2(-gamepad1.left_stick_x, -gamepad1.left_stick_y);
            joystickMagnitude = pow(gamepad1.left_stick_x, 2) + pow(gamepad1.left_stick_y, 2);
            turn = gamepad1.right_stick_x * Math.abs(gamepad1.right_stick_x);

            if(gamepad1.right_trigger< 0.1) {
                fr.setPower(Range.clip(joystickMagnitude * sin(PI / 4 + joystickAngle - robotHeading) - turn, -1, 1));
                fl.setPower(Range.clip(joystickMagnitude * sin(PI / 4 - joystickAngle + robotHeading) + turn, -1, 1));
                br.setPower(Range.clip(joystickMagnitude * sin(PI / 4 - joystickAngle + robotHeading) - turn, -1, 1));
                bl.setPower(Range.clip(joystickMagnitude * sin(PI / 4 + joystickAngle - robotHeading) + turn, -1, 1));
            }else{//speed Damper if right trigger pressed.
                fr.setPower(Range.clip(joystickMagnitude * sin(PI / 4 + joystickAngle - robotHeading) - turn, -1, 1)/3);
                fl.setPower(Range.clip(joystickMagnitude * sin(PI / 4 - joystickAngle + robotHeading) + turn, -1, 1)/3);
                br.setPower(Range.clip(joystickMagnitude * sin(PI / 4 - joystickAngle + robotHeading) - turn, -1, 1)/3);
                bl.setPower(Range.clip(joystickMagnitude * sin(PI / 4 + joystickAngle - robotHeading) + turn, -1, 1)/3);
            }
        }
    }
}