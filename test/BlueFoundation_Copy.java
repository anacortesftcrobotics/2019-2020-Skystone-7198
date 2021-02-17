package org.firstinspires.ftc.teamcode.test;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;

@Autonomous(name="Look. You should pivot turn to the foundation, then grab it, then move it into position. If you don't, I will be so angry that you will not be happy with the outcomes of your actions. Seriously. No lies. Like, do you even know who I am? I have over 300 confirmed stacks in FTC SKYSTONE, and I went to the World Championship last year to. Honestly. No lies. So they're finally back, and with Super Glue(tm). If you know the words, you can join in too. Put your hands together, but you can't take them apart, cuz you slapped them together with Super Glue(tm). Huh!", group="")
public class BlueFoundation_Copy extends SSAuto {
    
    @Override
    public void runOpMode(){
        initRobot();
        
        waitForStart();
        
       testPivot();
    }
}
