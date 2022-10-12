package org.firstinspires.ftc.teamcode.autonomous.redteam.leftStart;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;

import org.firstinspires.ftc.teamcode.autonomous.AbstractAutonomousOpModeRR;
import org.firstinspires.ftc.teamcode.autonomous.redteam.RedTeamPositions;

@Autonomous(name = "RedNoMoveLeft", group = "Red")
@Disabled
public class RedNoMove extends AbstractAutonomousOpModeRR {
    private Pose2d initPos = RedTeamPositions.INIT_POSTION_LEFT;

    private long delayMs = 5000;

    public RedNoMove() {
        super.TEAM_TYPE = AbstractAutonomousOpModeRR.RED_TEAM;
    }

    @Override
    protected void initOpModeBeforeStart() {
        super.initOpModeBeforeStart();
        drive.setPoseEstimate(initPos);

        telemetry.addData("Status", "Initialized");
        telemetry.update();
    }

    @Override
    protected void initOpModeAfterStart() {

    }

    @Override
    protected void executeOpMode() {
        //No op
    }
}
