package org.firstinspires.ftc.teamcode.autonomousRR.redteam;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.acmerobotics.roadrunner.trajectory.Trajectory;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.autonomousRR.AbstractAutonomousOpModeRR;

@Autonomous(name = "RedParkOnly", group = "Red")
public class RedParkOnly extends AbstractAutonomousOpModeRR {
    private Pose2d initPos = RedTeamPositions.WG2_START_POS;
    private Pose2d destPos = RedTeamPositions.PARK_ONLY_POSITION;
    private long delayMs = 5000;

    public RedParkOnly() {
        super.TEAM_TYPE = AbstractAutonomousOpModeRR.RED_TEAM;
    }

    @Override
    protected void initOpModeBeforeStart() {
        super.initOpModeBeforeStart();
        mecanumDriveRR.setPoseEstimate(initPos);

        telemetry.addData("Status", "Initialized");
        telemetry.update();
    }

    @Override
    protected void initOpModeAfterStart() {

    }

    @Override
    protected void executeOpMode() {
        //From Starting position to Case 0 drop zone
        Trajectory traj1 = mecanumDriveRR.trajectoryBuilder(initPos, 0)
                .splineToSplineHeading(destPos, 0)
                .build();

        Trajectory traj2 = mecanumDriveRR.trajectoryBuilder(traj1.end(), 0)
                .forward(6).build();

        this.sleep(delayMs);

        mecanumDriveRR.followTrajectory(traj1);
        mecanumDriveRR.followTrajectory(traj2);
    }
}
