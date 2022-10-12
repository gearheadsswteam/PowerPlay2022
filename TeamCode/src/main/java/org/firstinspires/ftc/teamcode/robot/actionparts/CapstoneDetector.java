package org.firstinspires.ftc.teamcode.robot.actionparts;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;

public class CapstoneDetector {

    public static String POSITION_A = "A";
    public static String POSITION_B = "B";
    public static String POSITION_C = "C";

    OpenCVObjectDetection objectDetector;

    public void intitalize(OpMode curOpmode){
//        objectDetector = new OpenCVObjectDetection();
//        objectDetector.initialize(curOpmode);
    }

    public String getPosition(){
       return POSITION_A;
    }
}
