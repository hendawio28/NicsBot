package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
@TeleOp
public class OdoTest extends OpMode {
    private final ElapsedTime initTimer = new ElapsedTime();
    private static final  Pose2D startingPosition = new Pose2D(DistanceUnit.INCH, -63, 24, AngleUnit.DEGREES, 0);
    private GoBildaPinpointDriver odo;
    private DcMotorEx turretMotor;
    private DcMotorEx flywheel = null;
    MecanumDrive mecanumDrive = new MecanumDrive();
    public final static double goalYPos = 70;
    public final static double goalXPos = 70;
    private final double ticksPerRev = 537.7;
    boolean set, done = false;
    double xPos, yPos, currentHeading, yDistance, xDistance, totalDistance, globalTargetAngle, turretTargetAngle;
    @Override
    public void init() {
        initTimer.reset();
        turretMotor = hardwareMap.get(DcMotorEx.class, "turret");
        flywheel = hardwareMap.get(DcMotorEx.class, "flywheel");
        flywheel.setMode(DcMotor.RunMode.RUN_USING_ENCODER);

        turretMotor.setPower(0.0);
        turretMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);
        turretMotor.setDirection(DcMotorSimple.Direction.REVERSE);
        mecanumDrive.init(hardwareMap);
        odo = hardwareMap.get(GoBildaPinpointDriver.class, "pinpoint");

        odo.setOffsets(168, -88, DistanceUnit.MM);

        odo.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);

        odo.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.REVERSED, GoBildaPinpointDriver.EncoderDirection.REVERSED);

        odo.recalibrateIMU();

        odo.setPosition(startingPosition);

        odo.update();
    }

    @Override
    public void init_loop() {
        if (initTimer.seconds() < 3) {
            turretMotor.setPower(0.3);
        }
        else {
            turretMotor.setPower(0.0);
            done = true;
        }
        if (!set && done) {
            turretMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
            turretMotor.setTargetPosition(0); // Good practice before switching to RUN_TO_POSITION
            turretMotor.setDirection(DcMotorSimple.Direction.FORWARD);
            turretMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);
            set = true;
            telemetry.addLine("Yay");
        }
    }

    @Override
    public void loop() {
        odo.update();
        flywheel.setVelocity(100);
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

        turretMotor.setTargetPosition((int)(Math.round(turretTargetAngle*(ticksPerRev /360)*3.8)));
        turretMotor.setPower(1.0);
        telemetry.addData("X Position", xPos);
        telemetry.addData("Y Position", yPos);
        telemetry.addData("RobotHeading", currentHeading);
        telemetry.addData("targetAngle", globalTargetAngle);
        telemetry.addData("TurretTargetAngle", turretTargetAngle);
        telemetry.addData("xDistance", xDistance);
        telemetry.addData("yDistance", yDistance);
        telemetry.addData("Distance to Goal",totalDistance);
        telemetry.addData("encoder reading",flywheel.getCurrentPosition());
        telemetry.addData("velocity", flywheel.getVelocity());

        telemetry.update();

    }
}