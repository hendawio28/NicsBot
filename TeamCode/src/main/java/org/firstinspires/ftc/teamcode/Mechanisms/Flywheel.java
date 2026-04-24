package org.firstinspires.ftc.teamcode.Mechanisms;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Flywheel {
    private double targetvelocity = 0;
    DcMotorEx flywheelMotor;
    public void init (HardwareMap hwMap) {
        flywheelMotor = hwMap.get(DcMotorEx.class, "flywheel");

        flywheelMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        flywheelMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
    }
    public void fly () {
        flywheelMotor.setVelocity(targetvelocity);
    }
    public boolean ready () {
        double currentVelocity = flywheelMotor.getVelocity();
        double velocityDifference = Math.abs(targetvelocity-currentVelocity);
        if (velocityDifference < 30) {
            return true;
        }
        else {
            return false;
        }
    }
}
