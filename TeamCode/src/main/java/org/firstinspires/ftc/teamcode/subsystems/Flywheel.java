package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Flywheel {
    DcMotorEx flywheelMotor;
    public void init (HardwareMap hwMap) {
        flywheelMotor = hwMap.get(DcMotorEx.class, "flywheel");

        flywheelMotor.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        flywheelMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
    }
    public void fly (double targetVelocity) {
        flywheelMotor.setVelocity(targetVelocity);
    }
//    public boolean ready () {
//        double currentVelocity = flywheelMotor.getVelocity();
//        double velocityDifference = Math.abs(targetvelocity-currentVelocity);
//        if (velocityDifference < 30) {
//            return true;
//        }
//        else {
//            return false;
//        }
//    }
}
