package org.firstinspires.ftc.teamcode.util;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import com.qualcomm.robotcore.hardware.ColorSensor;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.HardwareMap;


public class SSRobot {

    //Declare telemetry and hardware map classes
    Telemetry telemetry;
    HardwareMap hardwareMap;
    //DistanceSensor RightColorSensor_DistanceSensor;
    //ColorSensor RightColorSensor;
    
    //Declare components
    public SixWheelChassis chassis;
    public Arm arm;
    public PoseTracker poseTracker;
    public LightDriver lights;
    public SensorREVColorDistance cs;
    public VuMarkProcessorSkystone viewer;
    public LinearOpMode linearOpMode;
    
    //constructor for SSRobot
    public SSRobot (Telemetry telIn, HardwareMap mapIn, OpMode opModeIn){
        //set parameters to equal initial variables
        telemetry = telIn;
        hardwareMap = mapIn;
        if(opModeIn instanceof LinearOpMode){
            linearOpMode = (LinearOpMode) opModeIn;
        }
        //RightColorSensor_DistanceSensor = hardwareMap.get(DistanceSensor.class, "RightColorSensor");
        //RightColorSensor = hardwareMap.colorSensor.get("RightColorSensor");
        //RightColorSensor.enableLed(false);
        
        //Declare components inside consructor
        chassis = new SixWheelChassis(this);
        arm = new Arm(this);
        poseTracker = new PoseTracker(this);
        lights = new LightDriver(this);
        cs = new SensorREVColorDistance(this);
        viewer = new VuMarkProcessorSkystone(this);
    }
    
    /*public String toString() {
        return "RightColorSensor(argb): " + RightColorSensor.argb();
    }*/
    
}

