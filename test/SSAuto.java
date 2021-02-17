package org.firstinspires.ftc.teamcode.test;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import org.firstinspires.ftc.teamcode.util.*;

public abstract class SSAuto extends LinearOpMode{
    
    private SSRobot robot;
    
    String csColorLeft, csColorRight;
    String skystoneDetected = "None";
    
    public void initRobot() {
        robot = new SSRobot(telemetry, hardwareMap, this);
        robot.arm.setFoundation(0,1);
        
    }
    
    public void testPivot() {
        robot.chassis.rotate(-7,1);
        robot.chassis.controlSixWheel("forward", 60, 0.7);
        robot.chassis.controlSixWheel("forward", 20, 0.3);
        robot.arm.setFoundation(1,0);
        sleep(500);
        robot.chassis.rotate(250,1,8000);
        robot.chassis.controlSixWheel("forward", -20, 0.5);
        robot.chassis.rotate(40,0.8,3000);
        robot.arm.setFoundation(0,1);
        sleep(500);
        robot.chassis.controlSixWheel("forward", -10, 0.5);
        robot.chassis.rotate(-60,0.8,3000);
        robot.chassis.controlSixWheel("forward", 40, 0.8);
        robot.chassis.rotate(-110,0.8,3000);
        robot.chassis.controlSixWheel("forward", -20, 0.8);
        //robot.chassis.controlSixWheel("forward", 10, 0.8);
        robot.chassis.rotate(5,0.8);
        robot.chassis.controlSixWheel("forward", 100, 0.8);
    }
     public void testPivot2ElectricBoogaloo() {
        robot.chassis.rotate(7,1);
        robot.chassis.controlSixWheel("forward", 65, 0.7);
        robot.chassis.controlSixWheel("forward", 20, 0.3);
        robot.arm.setFoundation(1,0);
        sleep(750);
        robot.chassis.rotate(-250,1,8000);
        robot.chassis.controlSixWheel("forward", -20, 0.5);
        robot.chassis.rotate(-40,0.8,3000);
        robot.arm.setFoundation(0,1);
        sleep(500);
        robot.chassis.controlSixWheel("forward", -10, 0.5);
        robot.chassis.rotate(60,0.8,3000);
        robot.chassis.controlSixWheel("forward", 40, 0.8);
        robot.chassis.rotate(110,0.8,3000);
        robot.chassis.controlSixWheel("forward", -20, 0.8);
        //robot.chassis.controlSixWheel("forward", 10, 0.8);
        robot.chassis.rotate(-5,0.8);
        robot.chassis.controlSixWheel("forward", 100, 0.8);
    }
}

    /* public void path2(){
        //Go to foundation
        //robot.chassis.controlSixWheel("forward",40,0.7);
        robot.chassis.controlSixWheel("forward",5,0.7);
        robot.chassis.controlSixWheel("left",10,0.7);
        //robot.chassis.rotate(30,0.5);
        robot.chassis.controlSixWheel("forward",85,0.7);
        robot.chassis.controlSixWheel("right",10,0.7);
        //robot.chassis.rotate(-15,0.5);
        robot.chassis.controlSixWheel("forward",10,0.7);
        //grab foundation
        sleep(500);
        robot.arm.setFoundation(0.9,1);
        sleep(1000);
        //move foundation
        robot.chassis.controlSixWheel("forward",-130,1);
        //robot.chassis.controlSixWheel("right",60,1);
        //robot.chassis.controlSixWheel("forward",40,1);
        //let go of foundation
        sleep(500);
        robot.arm.setFoundation(0,1);
        sleep(1000);
        //escape
        robot.chassis.controlSixWheel("left",27,0.7);
        //robot.chassis.rotate(45,0.5);
        robot.chassis.controlSixWheel("forward",-25,0.7);
        robot.chassis.controlSixWheel("left",27,0.7);
        //robot.chassis.rotate(45,0.5);
        robot.chassis.controlSixWheel("forward",-25,0.7);
        robot.chassis.controlSixWheel("left",27,0.7);
        //robot.chassis.rotate(45,0.5);
        robot.chassis.controlSixWheel("forward",-35,0.7);
        robot.chassis.controlSixWheel("left",27,0.7);
        //robot.chassis.rotate(45,0.5);
        robot.chassis.controlSixWheel("forward",-35,0.7);
        robot.chassis.controlSixWheel("forward",15,0.7);
        robot.chassis.controlSixWheel("right",27,0.7);
        //robot.chassis.rotate(45,0.5);
        robot.chassis.controlSixWheel("forward",20,0.7);
        robot.chassis.controlSixWheel("right",27,0.7);
        //robot.chassis.rotate(45,0.5);
        robot.chassis.controlSixWheel("forward",-70,0.7);
    }*/

    /*public void path1(){
        //Go to foundation
        robot.chassis.controlSixWheel("forward",10,0.7);
        //robot.chassis.controlSixWheel("right",15,0.7);
        robot.chassis.rotate(-20,0.7);
        robot.chassis.controlSixWheel("forward",60,0.7);
        //robot.chassis.controlSixWheel("left",10,0.7);
        robot.chassis.rotate(20,0.7);
        robot.chassis.controlSixWheel("forward",40,0.7);
        //grab foundation
        sleep(500);
        robot.arm.setFoundation(0.9,1);
        sleep(1000);
        //move foundation
        robot.chassis.controlSixWheel("forward",-180,1);
        //let go of foundation
        sleep(500);
        robot.arm.setFoundation(0,1);
        sleep(1000);
        //escape
        //robot.chassis.controlSixWheel("right",27,0.7);
        robot.chassis.rotate(-45,0.7);
        robot.chassis.controlSixWheel("forward",-35,0.7);
        //robot.chassis.controlSixWheel("right",27,0.7);
        robot.chassis.rotate(-45,0.7);
        robot.chassis.controlSixWheel("forward",-35,0.7);
        //robot.chassis.controlSixWheel("right",27,0.7);
        robot.chassis.rotate(-45,0.7);
        robot.chassis.controlSixWheel("forward",-35,0.7);
        //robot.chassis.controlSixWheel("right",27,0.7);
        robot.chassis.rotate(-45,0.7);
        robot.chassis.controlSixWheel("forward",-35,0.7);
        robot.chassis.controlSixWheel("forward",15,0.7);
        //robot.chassis.controlSixWheel("left",30,0.7);
        robot.chassis.rotate(45,0.7);
        robot.chassis.controlSixWheel("forward",20,0.7);
        //robot.chassis.controlSixWheel("left",30,0.7);
        robot.chassis.rotate(45,0.7);
        robot.chassis.controlSixWheel("forward",-80,0.7);
    }*/