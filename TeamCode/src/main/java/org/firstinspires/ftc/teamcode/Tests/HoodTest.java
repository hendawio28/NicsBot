package org.firstinspires.ftc.teamcode.Tests;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.hardware.Servo;

public class HoodTest extends OpMode {
    Servo hood;
    @Override
    public void init() {
        hood = hardwareMap.get(Servo.class, "hood");
    }

    @Override
    public void loop() {
        hood.setPosition(0.076);
    }
}
