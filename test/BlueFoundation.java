package org.firstinspires.ftc.teamcode.test;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name="BlueFoundation", group="")
public class BlueFoundation extends SSAuto {
    
    @Override
    public void runOpMode(){
        initRobot();
        
        waitForStart();
        
        //pathBlueStones();
        //pathBlueFoundation();
        //pathBridge();
        testPivot2ElectricBoogaloo();
    }
}