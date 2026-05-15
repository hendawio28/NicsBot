package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Intake {
    private DcMotorEx intakeMotor;
    private double lastPower = 2;
    public static final double DEAD_ZONE = 0.5;

    public void init (HardwareMap hwMap) {
        intakeMotor = hwMap.get(DcMotorEx.class, "Intake");

        intakeMotor.setDirection(DcMotorEx.Direction.REVERSE);
        intakeMotor.setMode(DcMotorEx.RunMode.RUN_WITHOUT_ENCODER);

        intakeMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.FLOAT);
    }
    public void intake (double intakeSpeed) {
        double targetPower = 0.0;

        if (intakeSpeed >= DEAD_ZONE) {
            targetPower= 1.0;
        }
        else if (intakeSpeed <= -DEAD_ZONE) {
            targetPower = -1.0;
        }
        else {
            targetPower = 0.0;
        }
        if (targetPower !=lastPower) {
            intakeMotor.setPower(targetPower);
            lastPower = targetPower;
        }
}
}