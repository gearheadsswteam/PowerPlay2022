package org.firstinspires.ftc.teamcode.autonomousRR.redteam.auto1;

import com.acmerobotics.roadrunner.geometry.Pose2d;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

import org.firstinspires.ftc.teamcode.autonomousRR.AbstractAutonomousOpModeRR;

@Autonomous(name = "RedAutonomousModeRR1", group = "Red")
public class RedAutonomousModeRR1 extends AbstractAutonomousOpModeRR {

    Pose2d initPose = new Pose2d(0,0, 0);


    public RedAutonomousModeRR1() {
        super.TEAM_TYPE = AbstractAutonomousOpModeRR.RED_TEAM;
    }

    @Override
    protected void initOpModeBeforeStart() {
        super.initOpModeBeforeStart();
        mecanumDriveRR.setPoseEstimate(initPose);
        sleep(500);
        telemetry.addData("Status", "Initialized");
        telemetry.update();
    }


    @Override
    protected void initOpModeAfterStart() {

    }

    @Override
    protected void executeOpMode() {
        signal = robot.signalDetector.getCaseDetected();
        signal = 1;
        if (signal == 1) {
            telemetry.addData("Status", "Execute Case 1");
            telemetry.update();
            RedAutoCase1 case1 = new RedAutoCase1(mecanumDriveRR);
            case1.executeDrive();
            //System.out.println("Execute Case 1");
        } else if(signal == 2) {
            telemetry.addData("Status", "Execute Case 2");
            telemetry.update();
            RedAutoCase2 case2 = new RedAutoCase2(mecanumDriveRR);
            case2.executeDrive();
            //System.out.println("Execute Case 2");
        }  else if (signal == 3) {
            telemetry.addData("Status", "Execute Case 3");
            telemetry.update();
            RedAutoCase3 case3 = new RedAutoCase3(mecanumDriveRR);
            case3.executeDrive();
            //System.out.println("Execute Case 3");
        }
    }
}
