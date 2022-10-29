package org.firstinspires.ftc.teamcode.robot;

import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.vision.SignalDetector;


/**
 * This is NOT an opmode.
 * <p>
 * This class can be used to define all the specific hardware for a single robot.
 * In this case that robot is a Pushbot.
 * See PushbotTeleopTank_Iterative and others classes starting with "Pushbot" for usage examples.
 * <p>
 * This hardware class assumes the following device names have been configured on the robot:
 * Note:  All names are lower case and some have single spaces between words.
 * <p>
 * Motor channel:  Left  drive motor:        "left_drive"
 * Motor channel:  Right drive motor:        "right_drive"
 * Motor channel:  Manipulator drive motor:  "left_arm"
 * Servo channel:  Servo to open left claw:  "left_hand"
 * Servo channel:  Servo to open right claw: "right_hand"
 */
public class GearheadsMecanumRobotRR {

    //drive train for TelOp only
    public DcMotor fl_motor;
    public DcMotor fr_motor;
    public DcMotor rl_motor;
    public DcMotor rr_motor;

    //Gyro
    public BNO055IMU imu;

    public SignalDetector signalDetector;

    private LinearOpMode curOpMode = null;   //current opmode

    /* local OpMode members. */
    public HardwareMap hwMap = null;


    /* Constructor */
    public GearheadsMecanumRobotRR(LinearOpMode opMode) {
        this.curOpMode = opMode;
        hwMap = opMode.hardwareMap;
    }

    private void initVision() {
        signalDetector = new SignalDetector(hwMap);
        signalDetector.init();

    }

    /**
     * Initializes the Gyro
     *
     * @param calibrate
     */
    private void initGyro(boolean calibrate) {
        imu = hwMap.get(BNO055IMU.class, "gyro");
        BNO055IMU.Parameters parameters = new BNO055IMU.Parameters();

        parameters.mode = BNO055IMU.SensorMode.IMU;
        parameters.angleUnit = BNO055IMU.AngleUnit.DEGREES;
        parameters.accelUnit = BNO055IMU.AccelUnit.METERS_PERSEC_PERSEC;
        parameters.calibrationDataFile = "BNO055IMUCalibration.json"; // see the calibration sample opmode
        parameters.loggingEnabled = false;

        imu.initialize(parameters);
        calibrate = false;
        if (calibrate) {
            curOpMode.telemetry.addData("Mode", "calibrating...");
            curOpMode.telemetry.update();

            // make sure the imu gyro is calibrated before continuing.
            while (!curOpMode.isStopRequested() && !imu.isGyroCalibrated()) {
                curOpMode.sleep(10);
                curOpMode.idle();
            }
        }

        curOpMode.telemetry.addData("Mode", "waiting for start");
        curOpMode.telemetry.addData("imu calib status", imu.getCalibrationStatus().toString());
        curOpMode.telemetry.update();
    }


    /**
     * Initializes the drive train
     */
    private void initDriveMotors() {

        //DRIVING
        fl_motor = hwMap.dcMotor.get("fl");
        fr_motor = hwMap.dcMotor.get("fr");
        rl_motor = hwMap.dcMotor.get("bl");
        rr_motor = hwMap.dcMotor.get("br");


        //This is based on how motors have been mounted
        fr_motor.setDirection(DcMotor.Direction.REVERSE);
        rr_motor.setDirection(DcMotor.Direction.REVERSE);
        fl_motor.setDirection(DcMotor.Direction.FORWARD);
        rl_motor.setDirection(DcMotor.Direction.FORWARD);// BL motor works inverted...not sure why

        fr_motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rr_motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        fl_motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        rl_motor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        fr_motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rr_motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        fl_motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);
        rl_motor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

    }


    /* Initialize standard Hardware interfaces */
    public void initAutonomous(HardwareMap ahwMap, String teamType) {
        init(ahwMap);
        initGyro(true);
        initVision();
        //odoRetract.activateOdo();
    }

    /* Initialize standard Hardware interfaces */
    public void initTeleOp(HardwareMap ahwMap) {
        init(ahwMap);
        initGyro(true);
        initDriveMotors();
    }

    private void init(HardwareMap ahwMap) {
        // Save reference to Hardware map
        hwMap = ahwMap;

        initDriveMotors();

    }
}


