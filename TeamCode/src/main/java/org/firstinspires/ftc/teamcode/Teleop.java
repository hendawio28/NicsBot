package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import org.firstinspires.ftc.teamcode.subsystems.Flywheel;
import org.firstinspires.ftc.teamcode.subsystems.IndexerFSM;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.MecanumDrive;

@TeleOp
public class Teleop  extends OpMode {
    MecanumDrive mecanumDrive = new MecanumDrive();
    Intake intake = new Intake();
    IndexerFSM indexerFSM = new IndexerFSM();
    Flywheel flywheel = new Flywheel();

    private double lastTime;
    private boolean spin_for_intake, intakeOn, intakeWasOn= false;
    @Override
    public void init() {
        mecanumDrive.init(hardwareMap);
        intake.init(hardwareMap);
        //indexer.init(hardwareMap);
        indexerFSM.init(hardwareMap);
        flywheel.init(hardwareMap);

    }

    @Override
    public void loop() {
        //Time
        double time = getRuntime();

        ////Driving Code:
        //Assigning Variables
        double forward = -gamepad1.left_stick_y;
        double strafe = gamepad1.left_stick_x;
        double rotate = gamepad1.right_stick_x;
        //Actual Driving
        mecanumDrive.drive(forward, strafe, rotate);

        ////Intake Code

        //Assigning Variable
        double intakeSpeed = gamepad1.right_trigger - gamepad1.left_trigger;

        //Actual Intake
        intake.intake(intakeSpeed);

        ////Indexer Code

        //Assigning Variables
        boolean shoot = gamepad1.rightBumperWasPressed();
        intakeOn = intakeSpeed >= 0.3;
        if (intakeOn && !intakeWasOn) {
            spin_for_intake = true;
            lastTime = time;
        } else if (time >= lastTime+1.5 && intakeOn) {
            spin_for_intake = true;
            lastTime = time;
        }
        else {
            spin_for_intake = false;
        }

        //indexer.autoSpin(shoot, spin_for_intake);

        if (shoot) {
            indexerFSM.setTargetState(IndexerFSM.IndexerState.GO_TO_SHOOT);
        } else if (spin_for_intake) {
            indexerFSM.setTargetState(IndexerFSM.IndexerState.GO_TO_INTAKE);
        }
        indexerFSM.update();
        //flywheel.fly();

        telemetry.addData("Indexer State", indexerFSM.getStateName());
        telemetry.addData("Current Slot", indexerFSM.getCurrentSlot());
        telemetry.update();

        intakeWasOn = intakeOn;

    }
}
