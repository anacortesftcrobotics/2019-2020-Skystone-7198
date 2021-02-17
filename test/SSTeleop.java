package org.firstinspires.ftc.teamcode.test;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import org.firstinspires.ftc.teamcode.util.*;

@TeleOp(name = "SSTeleop", group = "")
public class SSTeleop extends OpMode{
    
    double DEAD_STICK = 0.2;
    double DEAD_WHEEL = 10;
    double MAX_VELOCITY = 380;
    double MAX_ACCEL = 15;
    double ARCADE_LIMIT = 0.7;
    
    double forwardLeft;
    double forwardRight;
    double drive;
    double turn;
    double Lift;
    double MainJoint;
    double SecondaryJoint;
    double stop1, stop2;
    
    boolean activeLift;
    
    SSRobot robot;
    //ColorSensor RightCS;
    //ColorSensor LeftCS;

    @Override
    public void init() {
        robot = new SSRobot(telemetry, hardwareMap, this);
        //RightCS = hardwareMap.colorSensor.get("RightColorSensor");
        //LeftCS = hardwareMap.colorSensor.get("LeftColorSensor");
        robot.viewer.initVu();
        robot.viewer.activate();
    }
    
    @Override
    public void loop() {
        //if(robot==null) {
        //    robot = new SSRobot(telemetry, hardwareMap,this);
        //}
        // Gets input, then runs the correct functions.
        getInput(); /* getInputEthan(); */
        robot.chassis.DriveAccelGud(forwardLeft, forwardRight);
        robot.arm.lift(Lift);
        if (!robot.arm.active) {
            robot.arm.moveArm(MainJoint, SecondaryJoint);
        } else {
            robot.arm.letGo();
        }
        robot.arm.levelCheck();
        robot.arm.setFoundation(stop1, stop2);
        // telemetry.addLine(robot.chassis.toString());
        // telemetry.addLine(robot.arm.toString());
        // telemetry.addLine(String.format("forwardLeft = %f", forwardLeft));
        // telemetry.addLine(robot.toString());
        // telemetry.addLine(String.format("CSLeftRed = %d", robot.cs.sensorColorLeft.red()));
        // telemetry.addLine(String.format("CSLeftGreen = %d", robot.cs.sensorColorLeft.green()));
        // telemetry.addLine(String.format("CSLeftBlue = %d", robot.cs.sensorColorLeft.blue()));
        // telemetry.addLine(String.format("CSLeftAlpha = %d", robot.cs.sensorColorLeft.alpha()));
        // telemetry.addLine(String.format("CSLeftColor = %s", robot.cs.CheckColor(robot.cs.sensorColorLeft)));
        // telemetry.addLine(String.format("CSRightRed = %d", robot.cs.sensorColorRight.red()));
        // telemetry.addLine(String.format("CSRightGreen = %d", robot.cs.sensorColorRight.green()));
        // telemetry.addLine(String.format("CSRightBlue = %d", robot.cs.sensorColorRight.blue()));
        // telemetry.addLine(String.format("CSRightAlpha = %d", robot.cs.sensorColorRight.alpha()));
        // telemetry.addLine(String.format("CSRightColor = %s", robot.cs.CheckColor(robot.cs.sensorColorRight)));
        // telemetry.update(); 
    }
    
    public void getInputTank(){
        forwardLeft = gamepad1.left_stick_y;
        forwardRight = -gamepad1.right_stick_y;
        lift();
        MainJoint = gamepad2.left_stick_y;
        SecJoint();
    }
    
    public void getInput(){
        if (getStick(gamepad1.left_stick_x) != 0 || getStick(gamepad1.left_stick_y) != 0) {
            drive = getStick(gamepad1.left_stick_y)*ARCADE_LIMIT;
            turn = getStick(gamepad1.left_stick_x)*ARCADE_LIMIT;
        } else if (getStick(gamepad1.left_trigger) != 0 || getStick(gamepad1.right_trigger) != 0) {
            drive = getStick(gamepad1.left_trigger)*0.5 - getStick(gamepad1.right_trigger)*0.5;
            //drive = gamepad1.left_trigger - gamepad1.right_trigger;
        } else {
            drive = getStick(gamepad1.right_stick_y);
            turn = 0;
        }
        
        /*if (getStick(gamepad1.left_trigger) != 0 || getStick(gamepad1.right_trigger) != 0) {
            drive = getStick(gamepad1.left_trigger)*0.5 - getStick(gamepad1.right_trigger)*0.5;
            //drive = gamepad1.left_trigger - gamepad1.right_trigger;
        } else if (getStick(gamepad1.left_stick_x) != 0 || getStick(gamepad1.left_stick_y) != 0) {
            drive = getStick(gamepad1.left_stick_y)*ARCADE_LIMIT;
            turn = getStick(gamepad1.left_stick_x)*ARCADE_LIMIT;
        } else {
            drive = getStick(gamepad1.right_stick_y);
            turn = 0;
        }*/
        
        forwardLeft    = -Range.clip(drive - turn, -1.0, 1.0) ;
        forwardRight   = -Range.clip(drive + turn, -1.0, 1.0) ;
        
        liftEthan();
        MainJoint = gamepad2.left_stick_y;
        SecJoint();
        if (gamepad2.a && !robot.arm.active) {
            robot.arm.active = true;
            robot.arm.timer.reset();
        }
        
        if (gamepad2.y && robot.arm.active) {
            robot.arm.active = false;
        }
        
        if(gamepad2.y) {
            robot.lights.teamColors();
            robot.viewer.findStone();
        }
        
        if(gamepad2.x) {
            robot.lights.rainbow();
        }
    }
    
    public double getStick(float stick) {
        if (Math.abs(stick)<=DEAD_STICK) {
            return 0;
        } else if (stick > 0) {
            return -Range.scale(stick, DEAD_STICK, 1, 0, 1);
        } else {
            return Range.scale(-stick, DEAD_STICK, 1, 0, 1);
        }
    }
    
    public void getInput3(){
        forwardLeft = gamepad1.right_stick_y;
        forwardRight = gamepad1.left_stick_x;
        lift();
        MainJoint = gamepad2.left_stick_y;
        SecJoint();
        if (gamepad2.a && !robot.arm.active) {
            robot.arm.letGo();
            //robot.arm.raise();
        }

    }
    
    public void getInputEthan() {
        /* added by SKJ for Ethan/Riley -
         * if right_stick - used for forward and back fast
        */
        if (gamepad1.left_trigger > 0.1 || gamepad1.right_trigger > 0.1) {
            forwardLeft = (gamepad1.left_trigger-gamepad1.right_trigger)*0.1;
            forwardRight = forwardLeft;
        } else if (Math.abs(gamepad1.right_stick_y) > 0.2) {
            forwardLeft = gamepad1.right_stick_y;
            if (gamepad1.right_stick_y > 0) {
                forwardRight = forwardLeft;
            } else {
                forwardLeft = -forwardLeft;
                forwardRight = forwardLeft; 
            }
        } /*else if (Math.abs(gamepad1.left_stick_y) > 0.2) {
            forwardLeft = gamepad1.left_stick_y;
            forwardRight = gamepad1.left_stick_y;  
        } */ /*else {
            //forwardLeft = gamepad1.left_stick_y;
            //forwardRight = gamepad1.right_stick_y;
            forwardLeft = -gamepad1.left_stick_x*0.3+gamepad1.left_stick_y;
            forwardRight = gamepad1.left_stick_x*0.3+gamepad1.left_stick_y;
        }*/
        lift();
        MainJoint = gamepad2.left_stick_y;
        SecJoint();
        if (gamepad2.a && !robot.arm.active) {
            robot.arm.letGo();
            //robot.arm.raise();
        }

    }

    //Checks the limit state and sets power to triggers for Player 2
    public void lift(){
        if (gamepad2.right_trigger > 0) {
            Lift = gamepad2.right_trigger;
        } else if (gamepad2.left_trigger > 0) {
            Lift = -gamepad2.left_trigger;
        } else {
            Lift = 0;
        }
        if (gamepad2.dpad_up) {
            robot.arm.levelUp();
        } else if (gamepad2.dpad_down) {
            robot.arm.levelDown();
        } else if (gamepad2.dpad_left) {
            robot.arm.levelMin();
        } else if (gamepad2.dpad_right) {
            robot.arm.levelMax();
        }
    }
    
    //Checks the limit state and sets power to triggers for Player 2
    public void liftEthan(){
        if (gamepad2.right_trigger > 0) {
            Lift = gamepad2.right_trigger;
        } else if (gamepad2.left_trigger > 0) {
            Lift = -gamepad2.left_trigger;
        } else {
            Lift = 0;
        }
        if (gamepad2.left_bumper) {
            robot.arm.setLevel(0);
        } else if (gamepad2.dpad_up) {
            robot.arm.setLevel(1);
        } else if (gamepad2.dpad_right) {
            robot.arm.setLevel(2);
        } else if (gamepad2.dpad_down) {
            robot.arm.setLevel(3);
        } else if (gamepad2.dpad_left) {
            robot.arm.setLevel(4);
        } else if (gamepad2.left_stick_button) {
            robot.arm.setLevel(5);
        }
    }
    
    public void SecJoint(){
        if(gamepad2.right_bumper)
            SecondaryJoint = 0.5;
        else if(!gamepad2.right_bumper)
            SecondaryJoint = 0;
            
        if(gamepad1.left_bumper){
            stop1 = 1;
        }
        else if(!gamepad1.left_bumper){
            stop1 = 0;
        }
        if(gamepad1.right_bumper){
            stop2 = 0;
        }
        else if(!gamepad1.right_bumper){
            stop2 = 1;
        }
    }
    
    public void liftArm() {
        
    }
}
