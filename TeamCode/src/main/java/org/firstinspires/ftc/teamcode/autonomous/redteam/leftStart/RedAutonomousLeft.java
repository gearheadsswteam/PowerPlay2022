package org.firstinspires.ftc.teamcode.autonomous.redteam.leftStart;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;

import org.firstinspires.ftc.teamcode.autonomous.redteam.RedTeamPositions;
import org.firstinspires.ftc.teamcode.drive.DriveConstants;
import org.firstinspires.ftc.teamcode.drive.SampleMecanumDrive;
import org.firstinspires.ftc.teamcode.robot.GearheadsMecanumRobotRR;
import org.firstinspires.ftc.teamcode.robot.actionparts.CapstoneArmSystem;
import org.firstinspires.ftc.teamcode.robot.actionparts.CapstoneDetector;
import org.firstinspires.ftc.teamcode.robot.actionparts.DeliveryArmSystem;
import org.firstinspires.ftc.teamcode.robot.actionparts.CarouselRotationSystem;
import org.firstinspires.ftc.teamcode.robot.actionparts.Intakesystem;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

import static java.lang.Math.PI;

@Disabled
class RedAutonomousLeft {

    private SampleMecanumDrive drive;
    private LinearOpMode currOpMode;
    private Pose2d initPos;
    private Pose2d lastPos;
    private String capStonePosition;
    DeliveryArmSystem deliveryArmSystem;
    CarouselRotationSystem carouselRotationSystem;
    Intakesystem intakesystem;
    CapstoneArmSystem capstoneArmSystem;

    public RedAutonomousLeft(SampleMecanumDrive mecanumDriveRR, GearheadsMecanumRobotRR gearheadsMecanumRobotRR, LinearOpMode currOpMode) {
        this.drive = mecanumDriveRR;
        this.currOpMode = currOpMode;
        this.initPos = RedTeamPositions.INIT_POSTION_LEFT;
        deliveryArmSystem = gearheadsMecanumRobotRR.deliveryArmSystem;
        carouselRotationSystem = gearheadsMecanumRobotRR.carouselRotationSystem;
        intakesystem = gearheadsMecanumRobotRR.intakesystem;
        capstoneArmSystem = gearheadsMecanumRobotRR.capstoneArmSystem;

    }

    public void setLastPos(Pose2d lastKnownPos) {
        this.lastPos = lastKnownPos;
    }

    public void setCapStonePosition(String capStonePosition) {
        this.capStonePosition = capStonePosition;
    }

    public void executeOpMode() {
        //Trajectoties from start position to Capstone pick up position
        Trajectory[] traj1 = {drive.trajectoryBuilder(RedTeamPositions.initPose)
                .lineToLinearHeading(RedTeamPositions.grabPose[0])
                .build(),
                drive.trajectoryBuilder(RedTeamPositions.initPose)
                        .lineToLinearHeading(RedTeamPositions.grabPose[1])
                        .build(),
                drive.trajectoryBuilder(RedTeamPositions.initPose)
                        .lineToLinearHeading(RedTeamPositions.grabPose[2])
                        .build()};


        //Trajectoties from Capstone pick up position to Shipping HUb drop position
        Trajectory[] traj2 = {drive.trajectoryBuilder(RedTeamPositions.grabPose[0])
                .splineTo(RedTeamPositions.dropPose.vec(), RedTeamPositions.dropPose.getHeading())
                .addTemporalMarker(1, 0, () -> {
                    deliveryArmSystem.bucketDown();
                })
                .build(),
                drive.trajectoryBuilder(RedTeamPositions.grabPose[1])
                        .addTemporalMarker(0, 0, () -> {
                            deliveryArmSystem.setLiftElevatorMedium();
                        })
                        .splineTo(RedTeamPositions.dropPose.vec(), RedTeamPositions.dropPose.getHeading())
                        .addTemporalMarker(1, 0, () -> {
                            deliveryArmSystem.bucketDown();
                        })
                        .build(),
                drive.trajectoryBuilder(RedTeamPositions.grabPose[2])
                        .addTemporalMarker(0, 0, () -> {
                            deliveryArmSystem.setLiftElevatorHigh();
                        })
                        .splineTo(RedTeamPositions.dropPose.vec(), RedTeamPositions.dropPose.getHeading())
                        .addTemporalMarker(1, 0, () -> {
                            deliveryArmSystem.bucketDown();
                        })
                        .build()};

        //Trajectories from  Shipping HUb drop position to Carousel
        Trajectory traj3 = drive.trajectoryBuilder(RedTeamPositions.dropPose, true)
                .addTemporalMarker(0.5, () -> {
                    deliveryArmSystem.setLiftElevatorLow();
                    deliveryArmSystem.bucketRest();
                    carouselRotationSystem.rotateAntiClockWise();
                })
                .splineTo(RedTeamPositions.spinnerPose.vec(), RedTeamPositions.spinnerPose.getHeading() + PI)
                .build();

        //Trajectories from  Carosel to Shipping Hub
        TrajectorySequence traj4 = drive.trajectorySequenceBuilder(RedTeamPositions.spinnerPose)
                .addTemporalMarker(0, 0, () -> {
                    intakesystem.startInTake();
                    carouselRotationSystem.stop();
                })
                .setTangent(-PI / 2)
                .setVelConstraint(SampleMecanumDrive.getVelocityConstraint(15, DriveConstants.MAX_ANG_VEL, DriveConstants.TRACK_WIDTH))
                .splineToSplineHeading(new Pose2d(-53, -61, PI / 2), 0)
                .splineToSplineHeading(new Pose2d(-40, -59, 2.2), 0)
                .resetVelConstraint()
                .splineToSplineHeading(RedTeamPositions.dropPose2, 0.9)
                .addTemporalMarker(1, -1, () -> {
                    deliveryArmSystem.setLiftElevatorHigh();
                    deliveryArmSystem.bucketUp();
                    intakesystem.stopInTake();
                })
                .addTemporalMarker(1, 0, () -> {
                    deliveryArmSystem.bucketDown();
                })
                .build();

        //Trajectories from  Shipping Hub to Storage Unit
        Trajectory traj5 = drive.trajectoryBuilder(RedTeamPositions.dropPose2)
                .lineToLinearHeading(RedTeamPositions.parkPose)
                .addTemporalMarker(0.5, () -> {
                    deliveryArmSystem.setLiftElevatorLow();
                    deliveryArmSystem.bucketRest();
                    ;
                })
                .build();


        //Going from start, grabbing capstone and going to shipping hub
        if (CapstoneDetector.POSITION_A.equals(capStonePosition)) {
            capstoneArmSystem.lowerArm();
            capstoneArmSystem.ungrabCapstone();
            drive.followTrajectory(traj1[0]);
            currOpMode.sleep(2000);
            capstoneArmSystem.grabCapstone();
            currOpMode.sleep(500);
            capstoneArmSystem.armUp();
            currOpMode.sleep(1000);
            drive.followTrajectory(traj2[0]);
            if (CapstoneDetector.POSITION_B.equals(capStonePosition)) {
                capstoneArmSystem.lowerArm();
                capstoneArmSystem.ungrabCapstone();
                drive.followTrajectory(traj1[1]);
                currOpMode.sleep(2000);
                capstoneArmSystem.grabCapstone();
                currOpMode.sleep(500);
                capstoneArmSystem.armUp();
                currOpMode.sleep(1000);
                drive.followTrajectory(traj2[1]);
            } else {
                capstoneArmSystem.lowerArm();
                capstoneArmSystem.ungrabCapstone();
                drive.followTrajectory(traj1[2]);
                currOpMode.sleep(2000);
                capstoneArmSystem.grabCapstone();
                currOpMode.sleep(500);
                capstoneArmSystem.armUp();
                currOpMode.sleep(1000);
                drive.followTrajectory(traj2[2]);
            }

            //Going from shipping hub to Carousel, pick up duck and then drop it off to the shipping hub
            currOpMode.sleep(500);
            drive.followTrajectory(traj3);
            currOpMode.sleep(2000);
            intakesystem.openIntakeGate();
            drive.followTrajectorySequence(traj4);
            currOpMode.sleep(500);
            drive.followTrajectory(traj5);
        }
    }
}
