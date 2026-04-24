package org.firstinspires.ftc.teamcode.Mechanisms;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
@TeleOp
public class OdoTest extends OpMode {
    private final ElapsedTime initTimer = new ElapsedTime();
    private static final  Pose2D startingPosition = new Pose2D(DistanceUnit.INCH, 24, -63, AngleUnit.DEGREES, 90);
    private GoBildaPinpointDriver odo;
    private DcMotor turretMotor;
    MecanumDrive mecanumDrive = new MecanumDrive();
    public final static double goalYPos = 70;
    public final static double goalXPos = 70;
    private final double tiksPerRev = 537.7;
    boolean set = false;
    double xPos, yPos, currentHeading, yDistance, xDistance, totalDistance, globalTargetAngle, turretTargetAngle;
    @Override
    public void init() {
        initTimer.reset();
        turretMotor = hardwareMap.get(DcMotor.class, "turret");
        turretMotor.setPower(0.0);
        turretMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        mecanumDrive.init(hardwareMap);
        odo = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");

        odo.setOffsets(168, -88, DistanceUnit.MM);

        odo.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);

        odo.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.FORWARD, GoBildaPinpointDriver.EncoderDirection.FORWARD);

        odo.recalibrateIMU();

        odo.setPosition(startingPosition);

        odo.update();
    }

    @Override
    public void init_loop() {
        if (initTimer.seconds() < 10) {
            turretMotor.setPower(0.3);
        }
        else {
            turretMotor.setPower(0.0);
        }
        if (!set) {
            turretMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            turretMotor.setTargetPosition(0); // Good practice before switching to RUN_TO_POSITION
            turretMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            set = true;
        }
    }

    @Override
    public void loop() {

        odo.update();

        double forward = -gamepad1.left_stick_y;
        double strafe = gamepad1.left_stick_x;
        double rotate = gamepad1.right_stick_x;

        mecanumDrive.drive(forward, strafe, rotate);

        xPos = odo.getPosX(DistanceUnit.INCH);
        yPos = odo.getPosY(DistanceUnit.INCH);
        currentHeading = odo.getHeading(AngleUnit.DEGREES);

        xDistance = goalXPos - xPos;
        yDistance = goalYPos - yPos;
        totalDistance = Math.hypot(xDistance, yDistance);

        globalTargetAngle = (Math.atan2(yDistance, xDistance))*(180/ Math.PI);
        turretTargetAngle = AngleUnit.normalizeDegrees(globalTargetAngle-currentHeading+90);

        if (turretTargetAngle > 180) {
            turretTargetAngle = 180;
        } else if (turretTargetAngle < 0) {
            turretTargetAngle = 0;

        }

        turretMotor.setTargetPosition((int)(Math.round(turretTargetAngle*(tiksPerRev/360)*3.8)));
        turretMotor.setPower(0.5);
        telemetry.addData("X Position", odo.getPosX(DistanceUnit.MM));
        telemetry.addData("Y Position", odo.getPosY(DistanceUnit.MM));
        telemetry.addData("RobotHeading", currentHeading);
        telemetry.addData("targetAngle", globalTargetAngle);
        telemetry.addData("TurretTargetAngle", turretTargetAngle);
        telemetry.addData("xDistance", xDistance);
        telemetry.addData("yDistance", yDistance);
        telemetry.addData("Distance to Goal",totalDistance);

        telemetry.update();

    }
}