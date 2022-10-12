package org.firstinspires.ftc.teamcode;
import static com.qualcomm.robotcore.hardware.DcMotorSimple.Direction.REVERSE;
import static java.lang.Math.*;
import static org.firstinspires.ftc.teamcode.ValueStorage.*;
import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.*;
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
@Autonomous (name = "AutonomousRed4" , group = "Red")
public class AutonomousRed4 extends LinearOpMode {
    Pose2d initPose = new Pose2d(8, -62, PI / 2);
    Pose2d[] grabPose = {new Pose2d(4, -48, PI / 2), new Pose2d(12, -48, PI / 2), new Pose2d(12, -46, 0.95)};
    Pose2d dropPose1 = new Pose2d(2, -35, 2.3); // preload
    Pose2d preDrop = new Pose2d(2, -50, 2);
    Pose2d dropPose2 = new Pose2d(-4, -40, 2); // fromn Warehouse
    Pose2d intakePose = new Pose2d(50, -65, PI);
    Pose2d intakePose2 = new Pose2d(54, -65, PI);
    Pose2d parkPose = new Pose2d(40, -65, PI);
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
    double INTAKE_SPEED = 1;
    double PRELOAD_DROP = 30;
    long ARM_DELAY_CASE_A = 1500;//Delay for the Arm and claw to get into position in Traj 1
    long ARM_DELAY_CASE_B = 1500;//Delay for the Arm and claw to get into position in Traj 1
    long ARM_DELAY_CASE_C = 1500;//Delay for the Arm and claw to get into position in Traj 1

    int SPEED_GOING_INTO_WAREHOUSE = 30;
    int SPEED_COMING_OUT_OF_WAREHOUSE = 30;

    //Trajectories
    TrajectorySequence[] traj1;
    TrajectorySequence[] traj2;
    TrajectorySequence traj3;
    TrajectorySequence traj4;
    TrajectorySequence traj5;
    TrajectorySequence traj6;
    TrajectorySequence traj7;

    @Override
    public void runOpMode() {
        initRobot();
        setupTrajForEverythingExceptPark();
        setupTrajForPark();
        runEverythingExceptPark();
        drive.followTrajectorySequence(traj7);
        ValueStorage.lastPose = drive.getPoseEstimate();
    }

    protected void runEverythingExceptPark() {
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
        ValueStorage.redMultiplier = 1;
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
            sleep(1000);
            drive.followTrajectorySequence(traj2[1]);
        } else {
            arm.setPosition(armDown);
            claw.setPosition(clawOpen);
            sleep(ARM_DELAY_CASE_C);
            drive.followTrajectorySequence(traj1[2]);

            claw.setPosition(clawClosed);
            sleep(500);
            arm.setPosition(armRest);
            sleep(1000);
            drive.followTrajectorySequence(traj2[2]);
        }
        sleep(800);
        bucket.setPosition(bucketRest);
        sleep(250);
        drive.followTrajectorySequence(traj3);
        drive.followTrajectorySequence(traj4);
        sleep(500);
        bucket.setPosition(bucketRest);
        sleep(250);
        drive.followTrajectorySequence(traj5);
        drive.followTrajectorySequence(traj6);
        sleep(500);
        bucket.setPosition(bucketRest);
        sleep(250);
    }

    protected void setupTrajForPark() {
        /**
         * Go the warehouse and take a cargo
         * TM1: Resets the lift , starts the intake, & gate up
         */

        traj7 = drive.trajectorySequenceBuilder(dropPose2)
                .setReversed(true)
                .addTemporalMarker(0.5, () -> {
                    lift.setTargetPosition(liftPositions[3]);
                    intake.setPower(0);
                })
                .splineToSplineHeading(new Pose2d(8, -59, PI), 2 + PI)
                .setVelConstraint(SampleMecanumDrive.getVelocityConstraint(30, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH))
                .splineToConstantHeading(new Vector2d(10, -65), 0)
                .lineTo(parkPose.vec())
                .build();
    }

    protected void setupTrajForEverythingExceptPark() {
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
                .splineTo(dropPose1.vec(), dropPose1.getHeading())
                .resetVelConstraint()
                .addTemporalMarker(1, 0, () -> {bucket.setPosition(bucketDown);})
                .build(),
                drive.trajectorySequenceBuilder(grabPose[1])
                        .setVelConstraint(SampleMecanumDrive.getVelocityConstraint(PRELOAD_DROP, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH))
                        .addTemporalMarker(0, 0, () -> {lift.setTargetPosition(liftPositions[1]);})
                        .splineTo(dropPose1.vec(), dropPose1.getHeading())
                        .resetVelConstraint()
                        .addTemporalMarker(1, 0, () -> {bucket.setPosition(bucketDown);})
                        .build(),
                drive.trajectorySequenceBuilder(grabPose[2])
                        .setVelConstraint(SampleMecanumDrive.getVelocityConstraint(PRELOAD_DROP, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH))
                        .addTemporalMarker(0, 0, () -> {lift.setTargetPosition(liftPositions[2]);})
                        .splineTo(dropPose1.vec(), dropPose1.getHeading())
                        .resetVelConstraint()
                        .addTemporalMarker(1, 0, () -> {bucket.setPosition(bucketDown);})
                        .build()};

        /**
         * Go the warehouse and take a cargo
         * TM1: Resets the lift , starts the intake, & gate up
         */

        traj3 = drive.trajectorySequenceBuilder(dropPose2)
                .setReversed(true)
                .addTemporalMarker(0.5, () -> {
                    lift.setTargetPosition(liftPositions[3]);
                    intake.setPower(INTAKE_SPEED);
                    gate.setPosition(gateUp);
                })
                .splineToSplineHeading(new Pose2d(8, -59, PI), 2 + PI)
                .setVelConstraint(SampleMecanumDrive.getVelocityConstraint(SPEED_GOING_INTO_WAREHOUSE, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH))
                .splineToConstantHeading(new Vector2d(10, -66), 0)
                .lineTo(intakePose.vec())
                .build();


        /**
         * Go from warehouse to the shipping hub for drop off
         * TM1: gate down 0.5 secomnds on route
         * TM2: Lift high position, i sec before end
         * TM3: Drop the bucket
         */
        traj4 = drive.trajectorySequenceBuilder(intakePose)
                .addTemporalMarker(1, () -> {
                    gate.setPosition(gateDown);
                })
                .setVelConstraint(SampleMecanumDrive.getVelocityConstraint(SPEED_COMING_OUT_OF_WAREHOUSE, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH))
                .splineTo(new Vector2d(13, -66), PI)
                .resetVelConstraint()
                .splineToConstantHeading(new Vector2d(9, -57), 2)
                //.lineToSplineHeading(preDrop)
                .splineToSplineHeading(preDrop, preDrop.getHeading())
                .setVelConstraint(SampleMecanumDrive.getVelocityConstraint(20, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH))
                .splineTo(dropPose2.vec(), dropPose2.getHeading())
                .resetVelConstraint()
                .addTemporalMarker(1, -1, () -> {
                    lift.setTargetPosition(liftPositions[2]);
                    bucket.setPosition(bucketUp);
                    intake.setPower(-1);
                })
                .addTemporalMarker(1, 0, () -> {
                    bucket.setPosition(bucketDown);
                })
                .build();


        /**
         * Go the warehouse and take a cargo
         * TM1: Resets the lift , starts the intake, & gate up
         */

        traj5 = drive.trajectorySequenceBuilder(dropPose2)
                .setReversed(true)
                .addTemporalMarker(0.5, () -> {
                    lift.setTargetPosition(liftPositions[3]);
                    intake.setPower(INTAKE_SPEED);
                    gate.setPosition(gateUp);
                })
                .splineToSplineHeading(new Pose2d(8, -59, PI), 2 + PI)
                .setVelConstraint(SampleMecanumDrive.getVelocityConstraint(SPEED_GOING_INTO_WAREHOUSE, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH))
                .splineToConstantHeading(new Vector2d(10, -65), 0)
                .lineTo(intakePose2.vec())
                .build();


        /**
         * Go from warehouse to the shipping hub for drop off
         * TM1: gate down 0.5 secomnds on route
         * TM2: Lift high position, i sec before end
         * TM3: Drop the bucket
         */

        traj6 = drive.trajectorySequenceBuilder(intakePose2)
                .addTemporalMarker(1, () -> {
                    gate.setPosition(gateDown);
                })
                .setVelConstraint(SampleMecanumDrive.getVelocityConstraint(SPEED_COMING_OUT_OF_WAREHOUSE, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH))
                .splineTo(new Vector2d(13, -66), PI)
                .resetVelConstraint()
                .splineToConstantHeading(new Vector2d(9, -58), 2)
                //.lineToSplineHeading(preDrop)
                .splineToSplineHeading(preDrop, preDrop.getHeading())
                .setVelConstraint(SampleMecanumDrive.getVelocityConstraint(20, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH))
                .splineTo(dropPose2.vec(), dropPose2.getHeading())
                .resetVelConstraint()
                .addTemporalMarker(1, -1, () -> {
                    lift.setTargetPosition(liftPositions[2]);
                    bucket.setPosition(bucketUp);
                    intake.setPower(-1);
                })
                .addTemporalMarker(1, 0, () -> {
                    bucket.setPosition(bucketDown);
                })
                .build();
    }

    protected void initRobot() {
        drive = new SampleMecanumDrive(hardwareMap);
        camera = new ShippingElementDetector(hardwareMap, 1);
        intake = hardwareMap.get(DcMotorEx.class, "intake");
        lift = hardwareMap.get(DcMotorEx.class, "lift");
        arm = hardwareMap.get(Servo.class, "arm");
        claw = hardwareMap.get(Servo.class, "claw");
        bucket = hardwareMap.get(Servo.class, "bucket");
        gate = hardwareMap.get(Servo.class, "gate");
        spinner = hardwareMap.get(CRServo.class, "spinner");
        intake.setDirection(REVERSE);
        drive.setPoseEstimate(initPose);
        lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        lift.setTargetPosition(liftPositions[3]);
        lift.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        camera.initialize();
    }
}
