package org.firstinspires.ftc.teamcode.util;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import java.util.concurrent.TimeUnit;
import java.lang.annotation.Target;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import com.qualcomm.robotcore.hardware.DcMotorSimple;

public class SixWheelChassis {

    //Define the robot
    SSRobot s;
    
    //Define motors
    DcMotorEx Right, Left;
    ColorSensor leftCS, RightCS;
    
    //Define motor power values
    double rightP, leftP;
    static double MAX_ROTATIONAL_VELOCITY = 380;
    static double MAX_ACCELERATION = 0.2;
    static double TICKS_PER_CM = 24;
    static double WHEEL_DISTANCE = 15 * 2.54; //The distance between the left and right wheels
    static double WHEEL_CIRCUMFERENCE = WHEEL_DISTANCE * Math.PI * 2;
    int TargetPosition, TargetPositionAngle;
    
    //CONSTRUCTOR!
    public SixWheelChassis(SSRobot _s){
        
        //set the reference
        s = _s;
        
        //Reference motors from hardware
        Left = s.hardwareMap.get(DcMotorEx.class, "left_drive");
        Right = s.hardwareMap.get(DcMotorEx.class, "right_drive");
        
        //Original Values: (1.213,0.121,0,12.13)
        Left.setVelocityPIDFCoefficients(1.213,0.121,0,12.13);
        //Original Value: 7.1
        Left.setPositionPIDFCoefficients(7.1);
        //Original Values: (1.213,0.121,0,12.13)
        Right.setVelocityPIDFCoefficients(1.213,0.121,0,12.13);
        //Original Value: 6.9
        Right.setPositionPIDFCoefficients(6.9);
        
        //REVERSE!
        Right.setDirection(DcMotorSimple.Direction.REVERSE);
        
        //Reset all encoders
        Left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        
        //Set Encoder
        Left.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Right.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        
        //Set power to zero
        Left.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        Right.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }
    
    public String toString() {
        return "Left: " + Left.getVelocity() +
               "\nRight: " + Right.getVelocity();
    }
    
    //Drive in a direction on one axis
    public void Drive(double leftForward, double rightForward){
        //get speed
        leftP = leftForward;
        rightP = rightForward;
        
        // Power the motors using a percent of the maximum velocity
        Left.setVelocity(MAX_ROTATIONAL_VELOCITY * leftP, AngleUnit.DEGREES);
        Right.setVelocity(MAX_ROTATIONAL_VELOCITY * rightP, AngleUnit.DEGREES);
        
    }
    
    public void setCoefficients(double p, double i, double d){
        //Original Values: (1.213,0.121,0,12.13)
        Left.setVelocityPIDFCoefficients(p,i,d,12.13);
        //Original Value: 7.1
        Left.setPositionPIDFCoefficients(7.1);
        //Original Values: (1.213,0.121,0,12.13)
        Right.setVelocityPIDFCoefficients(p,i,d,12.13);
        //Original Value: 6.9
        Right.setPositionPIDFCoefficients(6.9);
    }
    
    //Drive in a direction on one axis
    public void DriveAccelBad(double leftForward, double rightForward){
        //get speed
        leftP = (leftP+leftForward*MAX_ACCELERATION<=1 && leftP+leftForward*MAX_ACCELERATION>=-1)?leftP+leftForward*MAX_ACCELERATION:((leftP+leftForward*MAX_ACCELERATION>0)?1:0);
        rightP = (rightP+rightForward*MAX_ACCELERATION<=1 && rightP+rightForward*MAX_ACCELERATION>=-1)?rightP+rightForward*MAX_ACCELERATION:((rightP+rightForward*MAX_ACCELERATION>0)?1:0);
        
        // Power the motors using a percent of the maximum velocity
        ((DcMotorEx) Left).setVelocity(MAX_ROTATIONAL_VELOCITY * leftP, AngleUnit.DEGREES);
        ((DcMotorEx) Right).setVelocity(MAX_ROTATIONAL_VELOCITY * rightP, AngleUnit.DEGREES);
    }
    
    //Drive in a direction on one axis
    public void DriveAccelGud(double leftForward, double rightForward){
        //get speed
        if (leftForward > 0) {
            leftP += Math.min((leftForward-leftP),MAX_ACCELERATION);
        } else {
            leftP += Math.max((leftForward-leftP),-MAX_ACCELERATION);
        }
        
        if (rightForward > 0) {
            rightP += Math.min((rightForward-rightP),MAX_ACCELERATION);
        } else {
            rightP += Math.max((rightForward-rightP),-MAX_ACCELERATION);
        }
        
        // Power the motors using a percent of the maximum velocity
        Left.setVelocity(MAX_ROTATIONAL_VELOCITY * leftP, AngleUnit.DEGREES);
        Right.setVelocity(MAX_ROTATIONAL_VELOCITY * rightP, AngleUnit.DEGREES);
    }
    
    public void controlSixWheel(String type, int distCM, double power){
        Left.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        Right.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
        
        Left.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        Right.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        
        switch(type){
            case "forward":
                TargetPosition = (int) Math.floor(distCM * TICKS_PER_CM);
                Left.setTargetPosition(TargetPosition);
                Right.setTargetPosition(TargetPosition);
                Left.setPower(power);
                Right.setPower(power);
                break;
            case "right":
                TargetPosition = (int) Math.floor(distCM * TICKS_PER_CM);
                Left.setTargetPosition(-TargetPosition);
                Right.setTargetPosition(TargetPosition);
                Left.setPower(-power);
                Right.setPower(power);
                break;
            case "left":
                TargetPosition = (int) Math.floor(distCM * TICKS_PER_CM);
                Left.setTargetPosition(TargetPosition);
                Right.setTargetPosition(-TargetPosition);
                Left.setPower(power);
                Right.setPower(-power);
                break;
        }
        
        while((Left.isBusy() && Right.isBusy())){
            s.telemetry.addData("Type: ", type);
            s.telemetry.addData("Amount: ", TargetPosition);
            s.telemetry.addData("Left: ", "%d", Left.getCurrentPosition());
            s.telemetry.addData("Right: ", "%d", Right.getCurrentPosition());
            s.telemetry.update();
            if(Math.abs(Left.getCurrentPosition()) >= Math.abs(TargetPosition) - 15 || Math.abs(Right.getCurrentPosition()) >= Math.abs(TargetPosition) - 15){
                break;
            }
        }
        
        Left.setPower(0);
        Right.setPower(0);
    }
    
    // Rotates the robot to a certain position using the poseTracker's IMU
    public void rotate(double degrees, double power) {

        double leftPower, rightPower;
        
        setCoefficients(1.313,0.121,0);
        
        //Tell motors to run with encoders
        Left.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Right.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Resets the pose angle
        s.poseTracker.resetAngle();

        // Rotates to the specified position, reduces speed as it gets close
        if (degrees < 0) {
            setSimplePower(power, -power);
            while (s.poseTracker.getAngle() == degrees) {
                s.telemetry.addData("Angle:", s.poseTracker.getAngle());
                s.telemetry.update();
            }
            while (s.poseTracker.getAngle() > (degrees + 5) && s.linearOpMode.opModeIsActive()) {
                double correction = 1 - s.poseTracker.getAngle() / degrees;
                if (correction < 0.1) correction = 0.1;
                setSimplePower(power * correction, -power * correction);
                s.telemetry.addData("Angle:", s.poseTracker.getAngle());
                s.telemetry.addData("Correction:", correction);
                s.telemetry.update();
            }
        } else if (degrees > 0) {
            setSimplePower(-power, power);
            while (s.poseTracker.getAngle() < (degrees - 5) && s.linearOpMode.opModeIsActive()) {
                double correction = 1 - s.poseTracker.getAngle() / degrees;
                if (correction < 0.1) correction = 0.1;
                setSimplePower(-power * correction, power * correction);
                s.telemetry.addData("Angle:", s.poseTracker.getAngle());
                s.telemetry.addData("Correction:", correction);
                s.telemetry.update();
            }
        }
        setCoefficients(1.213,0.121,0);
        // Stop the robot
        setSimplePower(0, 0);

        // Reset the angle
        s.poseTracker.resetAngle();
    }
    
    // Rotates the robot to a certain position using the poseTracker's IMU
    public void rotate(double degrees, double power, double time) {

        double leftPower, rightPower;
        ElapsedTime rotTimer = new ElapsedTime();
        
        setCoefficients(1.313,0.121,0);
        
        //Tell motors to run with encoders
        Left.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Right.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Resets the pose angle
        s.poseTracker.resetAngle();

        // Rotates to the specified position, reduces speed as it gets close
        if (degrees < 0) {
            setSimplePower(power, -power);
            while (s.poseTracker.getAngle() == degrees) {
                s.telemetry.addData("Angle:", s.poseTracker.getAngle());
                s.telemetry.update();
            }
            while (s.poseTracker.getAngle() > (degrees + 5) && s.linearOpMode.opModeIsActive()) {
                double correction = 1 - s.poseTracker.getAngle() / degrees;
                if (correction < 0.1) correction = 0.1;
                setSimplePower(power * correction, -power * correction);
                s.telemetry.addData("Angle:", s.poseTracker.getAngle());
                s.telemetry.addData("Correction:", correction);
                s.telemetry.update();
                if (rotTimer.time(TimeUnit.MILLISECONDS) > time) break;
            }
        } else if (degrees > 0) {
            setSimplePower(-power, power);
            while (s.poseTracker.getAngle() < (degrees - 5) && s.linearOpMode.opModeIsActive()) {
                double correction = 1 - s.poseTracker.getAngle() / degrees;
                if (correction < 0.1) correction = 0.1;
                setSimplePower(-power * correction, power * correction);
                s.telemetry.addData("Angle:", s.poseTracker.getAngle());
                s.telemetry.addData("Correction:", correction);
                s.telemetry.update();
                if (rotTimer.time(TimeUnit.MILLISECONDS) > time) break;
            }
        }
        setCoefficients(1.213,0.121,0);
        // Stop the robot
        setSimplePower(0, 0);

        // Reset the angle
        s.poseTracker.resetAngle();
    }
    
    // Rotates the robot to a certain position using the poseTracker's IMU
    public void pivotBack(double degrees, double power, double ratio) {

        double leftPower, rightPower;
        
        power = -power;
        
        double p = 0.5;
        
        setCoefficients(1.313,0.121,0);
        
        //Tell motors to run with encoders
        Left.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Right.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Resets the pose angle
        s.poseTracker.resetAngle();

        // Rotates to the specified position, reduces speed as it gets close
        if (degrees > 0) {
            setSimplePower(power, power*ratio);
            while (s.poseTracker.getAngle() == degrees) {
                s.telemetry.addData("Angle:", s.poseTracker.getAngle());
                s.telemetry.update();
            }
            while (s.poseTracker.getAngle() < (degrees - 5)) {
                double correction = 1 - s.poseTracker.getAngle() / (degrees*p);
                if (correction < 0.2) correction = 0.2;
                setSimplePower(power * correction, power * correction * ratio);
                s.telemetry.addData("Angle:", s.poseTracker.getAngle());
                s.telemetry.addData("Correction:", correction);
                s.telemetry.update();
            }
        } else if (degrees < 0) {
            setSimplePower(power * ratio, power);
            while (s.poseTracker.getAngle() > (degrees + 5)) {
                double correction = 1 - s.poseTracker.getAngle() / (degrees*p);
                if (correction < 0.2) correction = 0.2;
                setSimplePower(power * correction * ratio, power * correction);
                s.telemetry.addData("Angle:", s.poseTracker.getAngle());
                s.telemetry.addData("Correction:", correction);
                s.telemetry.update();
            }
        }
        setCoefficients(1.213,0.121,0);
        // Stop the robot
        setSimplePower(0, 0);

        // Reset the angle
        s.poseTracker.resetAngle();
    }
    
    public void pivot(double degrees, double power, double ratio) {

        double leftPower, rightPower;
        
        setCoefficients(1.313,0.121,0);
        
        //Tell motors to run with encoders
        Left.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        Right.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        // Resets the pose angle
        s.poseTracker.resetAngle();

        // Rotates to the specified position, reduces speed as it gets close
        if (degrees < 0) {
            setSimplePower(power, power*ratio);
            while (s.poseTracker.getAngle() == degrees) {
                s.telemetry.addData("Angle:", s.poseTracker.getAngle());
                s.telemetry.update();
            }
            while (s.poseTracker.getAngle() > (degrees + 5)) {
                double correction = 1 - s.poseTracker.getAngle() / degrees;
                if (correction < 0.1) correction = 0.1;
                setSimplePower(power * correction, power * correction * ratio);
                s.telemetry.addData("Angle:", s.poseTracker.getAngle());
                s.telemetry.addData("Correction:", correction);
                s.telemetry.update();
            }
        } else if (degrees > 0) {
            setSimplePower(power * ratio, power);
            while (s.poseTracker.getAngle() < (degrees - 5)) {
                double correction = 1 - s.poseTracker.getAngle() / degrees;
                if (correction < 0.1) correction = 0.1;
                setSimplePower(power * correction * ratio, power * correction);
                s.telemetry.addData("Angle:", s.poseTracker.getAngle());
                s.telemetry.addData("Correction:", correction);
                s.telemetry.update();
            }
        }
        setCoefficients(1.213,0.121,0);
        // Stop the robot
        setSimplePower(0, 0);

        // Reset the angle
        s.poseTracker.resetAngle();
    }
    
    // Sets the power on the motors directly based on a left and right value
    public String setSimplePower(double lft, double rit) {
        Left.setPower(-lft);
        Right.setPower(-rit);
        return "Left: " + lft + " Right: " + rit;
    }
}