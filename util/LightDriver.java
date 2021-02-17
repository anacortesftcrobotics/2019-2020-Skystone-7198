package org.firstinspires.ftc.teamcode.util;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;


public class LightDriver {

    SSRobot s;
    
    RevBlinkinLedDriver leds;
    
    LightDriver(SSRobot _s) {
        s = _s;
        leds = s.hardwareMap.get(RevBlinkinLedDriver.class, "LEDs");
    }
    
    public void teamColors() {
        leds.setPattern(RevBlinkinLedDriver.BlinkinPattern.CP1_2_SPARKLE_2_ON_1);
    }
    
    public void rainbow() {
        leds.setPattern(RevBlinkinLedDriver.BlinkinPattern.RAINBOW_RAINBOW_PALETTE);
        
    }
}