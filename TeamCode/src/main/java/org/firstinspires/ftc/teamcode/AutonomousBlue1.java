package org.firstinspires.ftc.teamcode;
import static com.qualcomm.robotcore.hardware.DcMotorSimple.Direction.REVERSE;
import static java.lang.Math.*;
import static org.firstinspires.ftc.teamcode.ValueStorage.*;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;
import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;
@Autonomous (name = "AutonomousBlue1" , group = "Blue")
public class AutonomousBlue1 extends LinearOpMode {
    Pose2d initPose = BluePositions.initPose;
    Pose2d[] grabPose = BluePositions.grabPose;
    Pose2d dropPose = BluePositions.dropPose;
    Pose2d preDrop = new Pose2d(-31, 42, -0.7);
    Pose2d spinnerPose = BluePositions.spinnerPose;
    Pose2d parkPose = new Pose2d(-60, 33, PI);
    SampleMecanumDrive drive;
    ShippingElementDetector camera;
    DcMotorEx intake;
    DcMotorEx lift;
    Servo arm;
    Servo claw;
    Servo bucket;
    Servo gate;
    CRServo spinner;
    String caseDetected = "C";
    String caseSet = "C";
    int detectionFrames = 0;
    double SPINNER_SPEED = -0.25;
    double INTAKE_SPEED = 1;
    double PRELOAD_DROP = 30;

    long ARM_DELAY_CASE_A = 1500;//Delay for the Arm and claw to get into position in Traj 1
    long ARM_DELAY_CASE_B = 1500;//Delay for the Arm and claw to get into position in Traj 1
    long ARM_DELAY_CASE_C = 1500;//Delay for the Arm and claw to get into position in Traj 1

    TrajectorySequence[] traj1;
    TrajectorySequence[] traj2;
    Trajectory traj3;
    TrajectorySequence traj4;
    Trajectory traj5;


    @Override
    public void runOpMode() {
        initRobot();
        setupTrajFordeliverCargoAndDuck();
        setupTrajForpark();
        runEverythingExceptParkBlue();
        drive.followTrajectory(traj5);
        ValueStorage.lastPose = drive.getPoseEstimate();
    }

    /**
     * Set up for parking in the storage unit
     */
    void setupTrajForpark() {
        /**
         * Park in the storage unit
         */
        traj5 = drive.trajectoryBuilder(dropPose)
                .lineToLinearHeading(parkPose)
                .addTemporalMarker(0.5, () -> {lift.setTargetPosition(liftPositions[3]);})
                .build();
    }

    /**
     * Set up trajectories for cap grab, preload deliver, carousel, and duck deliver
     */
    void setupTrajFordeliverCargoAndDuck() {
        //Start to grab positons [0[ = postion A; [1] = postion B and [2] = position C
        traj1 = new TrajectorySequence[]{drive.trajectorySequenceBuilder(initPose)
                .setVelConstraint(SampleMecanumDrive.getVelocityConstraint(10, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH))
                .lineToLinearHeading(grabPose[0])
                .build(),
                drive.trajectorySequenceBuilder(initPose)
                        .setVelConstraint(SampleMecanumDrive.getVelocityConstraint(10, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH))
                        .lineToLinearHeading(grabPose[1])
                        .build(),
                drive.trajectorySequenceBuilder(initPose)
                        .setVelConstraint(SampleMecanumDrive.getVelocityConstraint(10, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH))
                        .lineToLinearHeading(grabPose[2])
                        .build()};

        //Grab position to dropping the preloaded box on the shipping hub. Array has the three start positions which are the grab positions
        traj2 = new TrajectorySequence[]{drive.trajectorySequenceBuilder(grabPose[0])
                .setVelConstraint(SampleMecanumDrive.getVelocityConstraint(PRELOAD_DROP, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH))
                .addTemporalMarker(0, 0, () -> {lift.setTargetPosition(liftPositions[0]);})
                .splineTo(dropPose.vec(), dropPose.getHeading())
                .resetVelConstraint()
                .addTemporalMarker(1, 0, () -> {bucket.setPosition(bucketDown);})
                .build(),
                drive.trajectorySequenceBuilder(grabPose[1])
                        .setVelConstraint(SampleMecanumDrive.getVelocityConstraint(PRELOAD_DROP, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH))
                        .addTemporalMarker(0, 0, () -> {lift.setTargetPosition(liftPositions[1]);})
                        .splineTo(dropPose.vec(), dropPose.getHeading())
                        .resetVelConstraint()
                        .addTemporalMarker(1, 0, () -> {bucket.setPosition(bucketDown);})
                        .build(),
                drive.trajectorySequenceBuilder(grabPose[2])
                        .setVelConstraint(SampleMecanumDrive.getVelocityConstraint(PRELOAD_DROP, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH))
                        .addTemporalMarker(0, 0, () -> {lift.setTargetPosition(liftPositions[2]);})
                        .splineTo(dropPose.vec(), dropPose.getHeading())
                        .resetVelConstraint()
                        .addTemporalMarker(1, 0, () -> {bucket.setPosition(bucketDown);})
                        .build()};

        /**
         * Drop off ot the Carousel
         * After 0.5 seconds reset the ift and start the spinner
         *
         */


        traj3 = drive.trajectoryBuilder(dropPose, true)
                .addTemporalMarker(0.5, () -> {
                    lift.setTargetPosition(liftPositions[3]);
                    spinner.setPower(SPINNER_SPEED);})
                .splineTo(spinnerPose.vec(), spinnerPose.getHeading() + PI)
                .build();

        /**
         * Intake the duck and go to the shipping hub
         * 1 second before the end move lift to highest position
         * Drop off the duck
         */
        traj4 = drive.trajectorySequenceBuilder(spinnerPose)
                .addTemporalMarker(0, 0, () -> {
                    gate.setPosition(gateDown);
                    spinner.setPower(0);})
                .setTangent(PI)
                .setVelConstraint(SampleMecanumDrive.getVelocityConstraint(20, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH))
                .splineToConstantHeading(new Vector2d(-45, 51), 0)
                .splineToSplineHeading(new Pose2d(-36, 61, -1), PI/2)
                .lineTo(new Vector2d(-60,61))
                .setVelConstraint(SampleMecanumDrive.getVelocityConstraint(10, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH))
                .lineToLinearHeading(new Pose2d(-62, 54, 0))
                .resetVelConstraint()
                .setTangent(0)
                .splineToSplineHeading(preDrop, -0.7)
                .setVelConstraint(SampleMecanumDrive.getVelocityConstraint(20, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH))
                .splineTo(dropPose.vec(), dropPose.getHeading())
                .resetVelConstraint()
                .addTemporalMarker(1, -1, () -> {lift.setTargetPosition(liftPositions[2]);
                    bucket.setPosition(bucketUp);
                    intake.setPower(0);})
                .addTemporalMarker(1, 0, () -> {bucket.setPosition(bucketDown);})
                .build();
    }

    /**
     * Initialize robot components
     */
    void initRobot() {
        drive = new SampleMecanumDrive(hardwareMap);
        camera = new ShippingElementDetector(hardwareMap, -1);
        intake = hardwareMap.get(DcMotorEx.class, "intake");
        lift = hardwareMap.get(DcMotorEx.class, "lift");
        arm = hardwareMap.get(Servo.class, "arm");
        claw = hardwareMap.get(Servo.class, "claw");
        bucket = hardwareMap.get(Servo.class, "bucket");
        gate = hardwareMap.get(Servo.class, "gate");
        spinner = hardwareMap.get(CRServo.class, "spinner");
        intake.setDirection(REVERSE);
        spinner.setDirection(REVERSE);
        drive.setPoseEstimate(initPose);
        lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lift.setTargetPosition(liftPositions[3]);
        lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        camera.initialize();
    }

    /**
     * Spin carousel and accelerate gradually
     */
    private void spinCarousel() {
        sleep(250);
        spinner.setPower(SPINNER_SPEED - 0.125);
        sleep(250);
        spinner.setPower(SPINNER_SPEED - 0.25);
        sleep(250);
        spinner.setPower(SPINNER_SPEED - 0.375);
        sleep(1500);
    }

    /**
     * Run all trajectories except parking trajectories
     */
    protected void runEverythingExceptParkBlue(){

        while (!isStarted() && !isStopRequested()) {
            if (camera.caseDetected() == caseDetected) {
                detectionFrames++;
            } else {
                caseDetected = camera.caseDetected();
                detectionFrames = 1;
            }
            if (detectionFrames >= cameraDetectionFrames) {
                caseSet = caseDetected;
            }
            telemetry.addData("Case:", caseSet);
            telemetry.update();
        }
        ValueStorage.lastIntakeState = 0;
        ValueStorage.lastLiftState = 0;
        ValueStorage.lastArmState = 0;
        ValueStorage.redMultiplier = -1;
        ValueStorage.lastPose = parkPose;
        camera.end();
        lift.setPower(1);
        bucket.setPosition(bucketUp);
        //Grab the Shipping element and drop off the box
        if (caseSet == "A") {
            arm.setPosition(armDown);
            claw.setPosition(clawOpen);
            sleep(ARM_DELAY_CASE_A);
            drive.followTrajectorySequence(traj1[0]);
            claw.setPosition(clawClosed);
            sleep(500);
            arm.setPosition(armRest);
            sleep(1000);
            drive.followTrajectorySequence(traj2[0]);
        } else if (caseSet == "B") {
            arm.setPosition(armDown);
            claw.setPosition(clawOpen);
            sleep(ARM_DELAY_CASE_B);
            drive.followTrajectorySequence(traj1[1]);
            claw.setPosition(clawClosed);
            sleep(500);
            arm.setPosition(armRest);
            sleep(800);
            drive.followTrajectorySequence(traj2[1]);
        } else {
            arm.setPosition(armDown);
            claw.setPosition(clawOpen);
            sleep(ARM_DELAY_CASE_C);
            drive.followTrajectorySequence(traj1[2]);
            claw.setPosition(clawClosed);
            sleep(500);
            arm.setPosition(armRest);
            sleep(500);
            drive.followTrajectorySequence(traj2[2]);
        }
        //Follow the above defined trjectories
        sleep(800);
        bucket.setPosition(bucketRest);
        sleep(250);
        drive.followTrajectory(traj3);
        intake.setPower(INTAKE_SPEED);
        spinCarousel();
        drive.followTrajectorySequence(traj4);
        sleep(500);
        bucket.setPosition(bucketRest);
        sleep(250);
    }

}
