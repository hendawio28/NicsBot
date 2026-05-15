package org.firstinspires.ftc.teamcode.Tests;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.Servo;
@TeleOp
public class TestHoodOpMode extends OpMode {
    Servo hood;
    @Override
    public void init() {
        hood = hardwareMap.get(Servo.class, "hood");
        telemetry.addData("Hood Position", hood.getPosition());
    }

    @Override
    public void init_loop() {
        hood.setPosition(0.0);
        telemetry.addData("Hood Position", hood.getPosition());
    }

    @Override
    public void loop() {
        hood.setPosition(0.5);
        telemetry.addData("Hood Position", hood.getPosition());
    }
}
