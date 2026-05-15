package org.firstinspires.ftc.teamcode.Tests;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.teamcode.Util.StartPosition;
import org.firstinspires.ftc.teamcode.subsystems.IndexerFSM2;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Localizer;
import org.firstinspires.ftc.teamcode.subsystems.MecanumDrive;
@TeleOp
public class ShootingTuning extends OpMode {
    DcMotorEx flywheel;
    Servo hood;
    Localizer localizer = new Localizer();
    MecanumDrive drive = new MecanumDrive();
    IndexerFSM2 indexerFSM2 = new IndexerFSM2();
    Intake intake = new Intake();
    public final static double BLUE_GOAL_X_POS = 0;
    public final static double RED_GOAL_X_POS = 144;
    public final static double GOAL_Y_POS = 144;
    double targetVelocity, increment, servoPos, servoIncrement = 0;

    @Override
    public void init() {
        localizer.init(hardwareMap, StartPosition.BLUE_BACK);
        drive.init(hardwareMap);
        indexerFSM2.init(hardwareMap);
        intake.init(hardwareMap);

        flywheel = hardwareMap.get(DcMotorEx.class, "flywheel");
        flywheel.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        flywheel.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.FLOAT);

        hood = hardwareMap.get(Servo.class, "hood");
        hood.setPosition(0.0);
        hood.setDirection(Servo.Direction.FORWARD);
    }

    @Override
    public void loop() {
        ////Driving Code:
        //Assigning Variables
        double forward = -gamepad1.left_stick_y;
        double strafe = gamepad1.left_stick_x;
        double rotate = gamepad1.right_stick_x;
        //Actual Driving
        drive.drive(forward, strafe, rotate);

        ////Intake Code:
        //Assigning Variable
        double intakeSpeed = gamepad1.right_trigger - gamepad1.left_trigger;
        //Actual Intake
        intake.intake(intakeSpeed);

        ////Indexer Code:
        //Assigning Variables
        boolean launchAll = gamepad1.yWasPressed();
        boolean shoot = gamepad1.dpad_right;
        boolean intakeOn = intakeSpeed >= Intake.DEAD_ZONE;
        indexerFSM2.setTargetState(launchAll, shoot, intakeOn);
        indexerFSM2.update();

        double xPos = localizer.getXPos();
        double yPos = localizer.getYPos();
        double xDistance = BLUE_GOAL_X_POS - xPos;
        double yDistance = GOAL_Y_POS - yPos;
        double distance = Math.hypot(xDistance, yDistance);
        if (gamepad1.rightBumperWasPressed()) {
            increment = +5;
        }
        else if (gamepad1.leftBumperWasPressed()) {
            increment = -5;
        }
        else  {
            increment = 0;
        }
        targetVelocity += increment;
        flywheel.setVelocity(targetVelocity);

        if (gamepad2.rightBumperWasPressed()) {
            servoIncrement = 0.001;
        }
        else if (gamepad2.leftBumperWasPressed()) {
            servoIncrement = -0.001;
        }
        servoPos += servoIncrement;
        if (servoPos > 1) {
            servoPos = 1;
        } else if ( servoPos < 0) {
            servoPos = 0;
        }
        hood.setPosition(servoPos);
        telemetry.addData("Distance", distance);
        telemetry.addData("TargetVelocity", targetVelocity);
        telemetry.addData("Current Velocity", flywheel.getVelocity());
        telemetry.addData("TargetServoPos", servoPos);
        telemetry.update();
    }
}
