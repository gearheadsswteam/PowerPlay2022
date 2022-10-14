package org.firstinspires.ftc.teamcode.autonomousRR.redteam;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.geometry.Vector2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.autonomousRR.AbstractAutonomousOpModeRR;
import org.firstinspires.ftc.teamcode.robot.GearheadsMecanumRobotRR;
import org.firstinspires.ftc.teamcode.trajectorysequence.TrajectorySequence;

@Autonomous(name = "RedAutonomousModeRR", group = "Red")
public class RedAutonomousModeRR extends AbstractAutonomousOpModeRR {

    Pose2d initPose = new Pose2d(0,0, 0);
    TrajectorySequence traj1;
    public RedAutonomousModeRR() {
        super.TEAM_TYPE = AbstractAutonomousOpModeRR.RED_TEAM;
    }

    @Override
    protected void initOpModeBeforeStart() {
        super.initOpModeBeforeStart();
        mecanumDriveRR.setPoseEstimate(initPose);
        traj1 = mecanumDriveRR.trajectorySequenceBuilder(initPose)
                .lineTo(new Vector2d(47, 0))
                .lineTo(new Vector2d(47, 47))
                .lineTo(new Vector2d(0, 47))
                .lineTo(new Vector2d(0, 0))
                .build();
        telemetry.addData("Status", "Initialized");
        telemetry.update();
    }

    @Override
    protected void initOpModeAfterStart() {

    }

    @Override
    protected void executeOpMode() {
        mecanumDriveRR.followTrajectorySequence(traj1);
    }
}
