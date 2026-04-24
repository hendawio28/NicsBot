package org.firstinspires.ftc.teamcode.Mechanisms;

import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

public class Kicker {
    Servo kicker;
    public void init(HardwareMap hwMap) {
        kicker = hwMap.get(Servo.class, "kicker");
        kicker.scaleRange(0.05, 0.3);
        kicker.setPosition(0.0); // Retract position
    }
    public void kick (double position) {
        kicker.setPosition(position);
    }
}
