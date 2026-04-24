package org.firstinspires.ftc.teamcode.Mechanisms;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;

public class Intake {
    private DcMotor intakeMotor;
    private double lastPower;

    public void init (HardwareMap hwMap) {
        intakeMotor = hwMap.get(DcMotor.class, "Intake");

        intakeMotor.setDirection(DcMotor.Direction.REVERSE);
        intakeMotor.setMode(DcMotor.RunMode.RUN_WITHOUT_ENCODER);

        intakeMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);
    }
    public void intake (double intakeSpeed) {
        double targetPower = 0.0;
        if (intakeSpeed >= 0.1) {
            targetPower= 1.0;
        }
        else if (intakeSpeed <= -0.1) {
            targetPower = -1.0;
        }
        else {
            targetPower = 0.0;
        }
        if (targetPower !=lastPower) {
            intakeMotor.setPower(targetPower);
        }

}
}