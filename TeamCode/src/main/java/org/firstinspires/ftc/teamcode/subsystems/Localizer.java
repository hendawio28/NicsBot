package org.firstinspires.ftc.teamcode.subsystems;

import com.pedropathing.ftc.FTCCoordinates;
import com.pedropathing.ftc.PoseConverter;
import com.pedropathing.geometry.PedroCoordinates;
import com.pedropathing.geometry.Pose;
import com.pedropathing.math.Vector;
import com.qualcomm.hardware.gobilda.GoBildaPinpointDriver;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

import org.firstinspires.ftc.robotcore.external.navigation.DistanceUnit;
import org.firstinspires.ftc.robotcore.external.navigation.Pose2D;
import org.firstinspires.ftc.teamcode.Util.StartPosition;

public class Localizer {
    private GoBildaPinpointDriver odo;
    private Pose2D ftcStartingPose;
    private double xPos, yPos, currentHeading;
    private boolean positionSet = false;
    private final ElapsedTime initTimer = new ElapsedTime();

    // Velocity tracking variables
    private Pose previousPedroPose = new Pose(0, 0, 0);
    private final ElapsedTime velocityTimer = new ElapsedTime();
    private double xVel = 0, yVel = 0;

    public void init (HardwareMap hwMap, StartPosition startPosition) {
        this.ftcStartingPose = PoseConverter.poseToPose2D(
                startPosition.pose, // Grab the pose from the Enum you passed in
                com.pedropathing.ftc.FTCCoordinates.INSTANCE
        );

        odo = hwMap.get(GoBildaPinpointDriver.class, "pinpoint");

        odo.setOffsets(168, -88, DistanceUnit.MM);

        odo.setEncoderResolution(GoBildaPinpointDriver.GoBildaOdometryPods.goBILDA_4_BAR_POD);

        odo.setEncoderDirections(GoBildaPinpointDriver.EncoderDirection.REVERSED, GoBildaPinpointDriver.EncoderDirection.REVERSED);

        odo.resetPosAndIMU();
        initTimer.reset();
        velocityTimer.reset();
        positionSet = false; // If you run it twice positionSet will be true so you need to set again as false
        previousPedroPose = startPosition.pose;
    }
    public void init_loop() {
        if (!positionSet && initTimer.seconds() > 0.5) {
            odo.setPosition(ftcStartingPose);
            odo.update();
            positionSet = true;
        }
    }
    public void update() {
        odo.update();
        Pose2D rawFtcPose = odo.getPosition();
        Pose pedroCords = PoseConverter.pose2DToPose(rawFtcPose, FTCCoordinates.INSTANCE)
                .getAsCoordinateSystem(PedroCoordinates.INSTANCE);
        //Assigning Variables
        xPos = pedroCords.getX();
        yPos = pedroCords.getY();
        currentHeading = Math.toDegrees(pedroCords.getHeading());

        double deltaTime = velocityTimer.seconds();
        velocityTimer.reset();

        if (deltaTime > 0) {
            xVel = (pedroCords.getX() - previousPedroPose.getX()) / deltaTime;
            yVel = (pedroCords.getY() - previousPedroPose.getY()) / deltaTime;
        }

        previousPedroPose = new Pose(pedroCords.getX(), pedroCords.getY(), pedroCords.getHeading());

    }
    public double getXPos() {
        return xPos;
    }

    public double getYPos() {
        return yPos;
    }

    public double getHeading() {
        return currentHeading;
    }

    public double getXVelocity() { return xVel; }
    public double getYVelocity() { return yVel; }

    public Vector getVelocity() {
        return new Vector(xVel, yVel);
    }

}
