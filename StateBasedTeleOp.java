package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import com.qualcomm.robotcore.util.Range;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.Hardware;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Blinker;
import com.qualcomm.robotcore.hardware.Servo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.Gyroscope;
import com.qualcomm.robotcore.hardware.DigitalChannel;

@TeleOp
public class StateBasedTeleOp extends OpMode{
    private DcMotor left_drive;
    private DcMotor right_drive;
    
    double DEAD_STICK = 0.2;
    double DEAD_WHEEL = 10;
    double MAX_VELOCITY = 380;
    double MAX_ACCEL = 15;
    double ARCADE_LIMIT = 0.7;
    
    double leftPower;
    double rightPower;
    double drive;
    double turn;

    @Override
    public void init() {
        left_drive = hardwareMap.dcMotor.get("left_drive");
        right_drive = hardwareMap.dcMotor.get("right_drive");
        
        left_drive.setDirection(DcMotor.Direction.REVERSE);
        right_drive.setDirection(DcMotor.Direction.FORWARD);
        
        left_drive.setDirection(DcMotor.Direction.REVERSE);
        right_drive.setDirection(DcMotor.Direction.FORWARD);
        
        left_drive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        right_drive.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
    }
    
    @Override
    public void loop() {
        getInput();
        
        
        
        ((DcMotorEx)left_drive).setVelocity(leftPower*MAX_VELOCITY, AngleUnit.DEGREES);
        ((DcMotorEx)right_drive).setVelocity(rightPower*MAX_VELOCITY, AngleUnit.DEGREES);
        
        telemetry.addData("speed", ((DcMotorEx)left_drive).getVelocity(AngleUnit.DEGREES));
        telemetry.update();
        
    }
    
    public void getInput(){
        if (getStick(gamepad1.left_stick_x) != 0 || getStick(gamepad1.left_stick_y) != 0) {
            drive = getStick(gamepad1.left_stick_y)*ARCADE_LIMIT;
            turn = getStick(gamepad1.left_stick_x)*ARCADE_LIMIT;
        } else {
            drive = getStick(gamepad1.right_stick_y);
            turn = 0;
        }
        
        leftPower    = Range.clip(drive - turn, -1.0, 1.0) ;
        rightPower   = Range.clip(drive + turn, -1.0, 1.0) ;
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
}