package org.firstinspires.ftc.teamcode;

import android.graphics.Color;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DistanceSensor;
import org.firstinspires.ftc.robotcore.external.JavaUtil;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;

@TeleOp(name = "TestCS (Blocks to Java)", group = "")
public class TestCS extends LinearOpMode {

  private DistanceSensor RightColorSensor_DistanceSensor;
  private ColorSensor RightColorSensor;

  /**
   * This function is executed when this Op Mode is selected from the Driver Station.
   */
  @Override
  public void runOpMode() {
    RightColorSensor_DistanceSensor = hardwareMap.get(DistanceSensor.class, "RightColorSensor");
    RightColorSensor = hardwareMap.colorSensor.get("RightColorSensor");

    // This op mode demonstrates the color and distance features of the REV sensor.
    telemetry.addData("Color Distance Example", "Press start to continue...");
    telemetry.update();
    waitForStart();
    if (opModeIsActive()) {
      // Put run blocks here.
      while (opModeIsActive()) {
        // Display distance info.
        telemetry.addData("Dist to tgt (cm)", RightColorSensor_DistanceSensor.getDistance(DistanceUnit.CM));
      }
    }
  }
}
