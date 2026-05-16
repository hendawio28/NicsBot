package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import org.firstinspires.ftc.teamcode.Util.StartPosition;
import org.firstinspires.ftc.teamcode.subsystems.IndexerFSM2;
import org.firstinspires.ftc.teamcode.subsystems.Intake;
import org.firstinspires.ftc.teamcode.subsystems.Localizer;
import org.firstinspires.ftc.teamcode.subsystems.MecanumDrive;
import org.firstinspires.ftc.teamcode.subsystems.Turret;
@TeleOp
public class TestBlueTeleop extends OpMode {
    Localizer localizer = new Localizer();
    Turret turret = new Turret();
    MecanumDrive drive = new MecanumDrive();
    Intake intake = new Intake();
    IndexerFSM2 indexerFSM = new IndexerFSM2();
    @Override
    public void init() {
        localizer.init(hardwareMap, StartPosition.BLUE_BACK);
        turret.init(hardwareMap);
        drive.init(hardwareMap);
        intake.init(hardwareMap);
        indexerFSM.init(hardwareMap);


    }

    @Override
    public void init_loop() {

        turret.init_loop(telemetry);
        localizer.init_loop();
    }

    @Override
    public void loop() {
        //// Localization:
        //Fetching Data
        localizer.update();
        //Assigning Variables
        double xPos = localizer.getXPos();
        double yPos = localizer.getYPos();
        double heading = localizer.getHeading();

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

        ////Turret Code:
        turret.trackBlue(xPos, yPos, heading);
        turret.turretStats(telemetry);

        ////Indexer Code:
        //Assigning Variables
        boolean launchAll = gamepad1.yWasPressed();
        boolean shoot = gamepad1.rightBumperWasPressed();
        boolean intakeOn = intakeSpeed >= Intake.DEAD_ZONE;
        indexerFSM.setTargetState(launchAll, shoot, intakeOn);
        indexerFSM.update();
        telemetry.addData("Xpos", xPos);
        telemetry.addData("yPos", yPos);
        telemetry.addData("Current Heading", heading);
    }
}
