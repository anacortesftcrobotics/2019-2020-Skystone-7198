package org.firstinspires.ftc.teamcode.util;

import com.qualcomm.robotcore.hardware.DcMotor;
import java.util.logging.Level;
import java.util.concurrent.TimeUnit;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotorSimple.Direction;


public class Arm {
    //Define the robot
    SSRobot s;
    
    //Define motors
    DcMotor lift;
    Servo grabFoundationLeft, grabFoundationRight, grabber;
    //CRServo grabberArm;
    DcMotor grabberArm;
    DigitalChannel limit;
    public ElapsedTime timer;
    
    //Define motor power values
    double liftP;
    double grabberArmPos;
    double grabFoundationPos, grabberPos;
    static double MAX_ROTATIONAL_VELOCITY = 300;
    static double LEVEL_HEIGHT = 1600;
    static int MAX_LEVEL = 5;
    public boolean active = false;
    public int currentLevel = 0;
    public int newLevel = 0;
    boolean lAct;
    
    //CONSTRUCTOR!
    public Arm(SSRobot _s){
        
        //set the reference
        s = _s;
        
        //Reference motors from hardware
        //lift = s.hardwareMap.dcMotor.get("lift_motor");
        lift = s.hardwareMap.dcMotor.get("lift");
        
        //Reset all encoders
        lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        
        //Set Encoder
        lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        
        //Set power to zero
        lift.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        
        //Reference and set limit to input
        limit = s.hardwareMap.digitalChannel.get("limit1");
        limit.setMode(DigitalChannel.Mode.INPUT);
        
        //Reference continuous rotation servo from hardware
        //grabberArm = s.hardwareMap.crservo.get("grabber_arm");
        grabberArm = s.hardwareMap.dcMotor.get("grabber_arm");
        grabberArm.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        grabberArm.setPower(0);
        
        grabber = s.hardwareMap.servo.get("grabber");
        grabber.setPosition(0.0);
        
        grabFoundationLeft = s.hardwareMap.servo.get("grab_foundation_left");
        grabFoundationRight = s.hardwareMap.servo.get("grab_foundation_right");
        
        grabFoundationLeft.setPosition(0);
        grabFoundationRight.setPosition(1);
        
        timer = new ElapsedTime();
        
    }
    
    public void lift(double power){
        //get speed
        liftP = power;
        
        if (liftP < 0 && checkLimit()) {
            liftP = 0;
            if (lift.getCurrentPosition() != 0) {
                //Reset all encoders
                lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                
                //Set Encoder
                lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                
                currentLevel = 0;
                newLevel = 0;
            }
        }
        
        if (!lAct) {
            ((DcMotorEx) lift).setVelocity((MAX_ROTATIONAL_VELOCITY * liftP), AngleUnit.DEGREES);
            currentLevel = (int)(lift.getCurrentPosition()/LEVEL_HEIGHT);
            newLevel = currentLevel;
        }
    }
    
    //sets the power of the main joint, the distance apart the claw is, 
    //and distance from the ground the stopper is.
    public void moveArm(double Power, double width){
        grabberArmPos = Power;
        grabberPos = width;
        //grabFoundationPos = distance;
        
        grabberArm.setPower(grabberArmPos);
        grabber.setPosition(grabberPos);
        //grabFoundation.setPosition(grabFoundationPos);
    }
    
    public void setFoundation(double positionLeft, double positionRight) {
        grabFoundationLeft.setPosition(positionLeft);
        grabFoundationRight.setPosition(positionRight);
    }
    
    public boolean checkLimit(){
        //get limit
        return limit.getState();
    }
    
    public void letGo() {
        if (timer.time(TimeUnit.MILLISECONDS) < 500 && active) {
            grabber.setPosition(0.5);
        } else if (timer.time(TimeUnit.MILLISECONDS) < 800 && active) {
            grabberArm.setPower(-1);
        } else if (timer.time(TimeUnit.MILLISECONDS) < 1300 && active) {
            active = false;
        }
    }
    
    public void levelDown(){
        if (currentLevel != 0 && !lAct) {
            lAct = true;
            newLevel--;
        }
    }
    
    public void levelUp() {
        if (currentLevel < MAX_LEVEL && !lAct) {
            lAct = true;
            newLevel++;
        }
    }
    
    public void levelMin(){
        if (newLevel != 0 && !lAct) {
            lAct = true;
            newLevel = 0;
        }
    }
    
    public void levelMax(){
        if (newLevel != MAX_LEVEL && !lAct) {
            lAct = true;
            newLevel = MAX_LEVEL;
        }
    }
    
    public void setLevel(int level) {
        if (newLevel != level && level >= 0 && level <= MAX_LEVEL && !lAct) {
            lAct = true;
            newLevel = level;
        }
    }
    
    public void levelCheck() {
        double speed = lift.getPower();
        if (lAct) {
            if (speed < 0 && checkLimit()) {
                lift.setPower(0);
                lAct = false;
                if (lift.getCurrentPosition() != 0) {
                    //Reset all encoders
                    lift.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                    
                    //Set Encoder
                    lift.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
                    
                    currentLevel = 0;
                    newLevel = 0;
                }
            }
            if (newLevel>currentLevel && lift.getPower() != 1) {
                lift.setPower(1);
            } else if (newLevel<currentLevel && lift.getPower() != -1) {
                lift.setPower(-1);
            }
            currentLevel = (int)(lift.getCurrentPosition()/LEVEL_HEIGHT);
            if (lift.getPower() < 0 && lift.getCurrentPosition() <= newLevel*LEVEL_HEIGHT) {
                lift.setPower(0);
                lAct = false;
                currentLevel = newLevel;
            } else if (lift.getPower() > 0 && lift.getCurrentPosition() >= newLevel*LEVEL_HEIGHT) {
                lift.setPower(0);
                lAct = false;
                currentLevel = newLevel;
            }
        }
    }
    
    public String toString (){
        return "lift power: " + liftP + 
               "\nLimit State: " + limit.getState();
    }
}