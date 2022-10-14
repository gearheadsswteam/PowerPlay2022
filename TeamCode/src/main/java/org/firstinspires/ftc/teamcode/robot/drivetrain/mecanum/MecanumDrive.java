package org.firstinspires.ftc.teamcode.robot.drivetrain.mecanum;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotor;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.AxesOrder;
import org.firstinspires.ftc.robotcore.external.navigation.AxesReference;

public class MecanumDrive {

    DcMotor fl_motor, fr_motor, rl_motor, rr_motor;
    private double x, y, rotation;

    private BNO055IMU gyro;
    private double angleConstant = 0.04;
    private double angleMax = 0.3;

    private String dataString;

    public MecanumDrive(DcMotor fl_motor, DcMotor fr_motor, DcMotor rl_motor, DcMotor rr_motor) {
        this.fl_motor = fl_motor;
        this.fr_motor = fr_motor;
        this.rl_motor = rl_motor;
        this.rr_motor = rr_motor;
    }

    public MecanumDrive(DcMotor fl_motor, DcMotor fr_motor, DcMotor rl_motor, DcMotor rr_motor, BNO055IMU gyro) {
        this.fl_motor = fl_motor;
        this.fr_motor = fr_motor;
        this.rl_motor = rl_motor;
        this.rr_motor = rr_motor;
        this.gyro = gyro;
        //this.gyro.calibrate(); //Not sure if this is needed
    }

    public void move(double x, double y, double rotation) {
        this.x = x;
        this.y = -y;
        this.rotation = rotation;
        dataString = "\nx: " + x + "\n y: " + y + "\n rot: " + rotation;
        doit();
    }

    public void stopRobot() {
        fl_motor.setPower(0);
        fr_motor.setPower(0);
        rl_motor.setPower(0);
        rr_motor.setPower(0);
    }

    private void angleRotation(int angle) {
        if (gyro == null) {
            return;
        }

        //Converts to values between 0 and 359 to match modernrobotics gyro
        angle = angle % 360;
        if (angle < 0) {
            angle = 360 + angle;
        }

        int heading = (int) gyro.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS).firstAngle;
        int dist = angle - heading; //ASSUMES: rotation clockwise is positive

        //Reverse direction is other side is closer
        if (Math.abs(dist) > 180) {
            if (dist < 0) {
                dist = 360 + dist;
            } else if (dist > 0) {
                dist = 360 - dist;
            }
        }

        if (Math.abs(dist) > 1) {
            rotation = dist * angleConstant; //Proportional
            rotation = Math.max(Math.min(rotation, angleMax), -angleMax); //Floor and ceiling
        }
    }

    private boolean onAngle(int angle) {
        int heading = (int) gyro.getAngularOrientation(AxesReference.INTRINSIC, AxesOrder.ZYX, AngleUnit.RADIANS).firstAngle;
        int dist = angle - heading;

        return Math.abs(dist) <= 2;
    }

    public void initEncoderMode() {

        fl_motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        fr_motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rl_motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        rr_motor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        setRunUsingEncoderMode();
    }

    public void setRunUsingEncoderMode() {
        fl_motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        fr_motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rl_motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        rr_motor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
    }

    public String getDataString() {
        return dataString;
    }

    public void doit() {
        double fl_speed = x + y + rotation;
        double fr_speed = -x + y - rotation;
        double rl_speed = -x + y + rotation;
        double rr_speed = x + y - rotation;

        double max = 1;
        max = Math.max(Math.abs(max), Math.abs(fl_speed));
        max = Math.max(Math.abs(max), Math.abs(fr_speed));
        max = Math.max(Math.abs(max), Math.abs(rl_speed));
        max = Math.max(Math.abs(max), Math.abs(rr_speed));

        if (max > 1) {
            fl_speed /= max;
            fr_speed /= max;
            rl_speed /= max;
            rr_speed /= max;
        }

        fl_motor.setPower(fl_speed);
        fr_motor.setPower(fr_speed);
        rl_motor.setPower(rl_speed);
        rr_motor.setPower(rr_speed);

        //lastSpeeds = "fl: "+fl_speed+" fr: "+fr_speed+" rl: "+rl_speed+" rr: "+rr_speed;
        dataString = "x: " + x + " y: " + y + " rot: " + rotation;

        x = 0;
        y = 0;
        rotation = 0;
    }
}