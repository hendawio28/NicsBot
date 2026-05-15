//package org.firstinspires.ftc.teamcode.Tests;
//
//import com.pedropathing.geometry.Pose;
//import com.qualcomm.robotcore.util.Range;
//
//import org.firstinspires.ftc.teamcode.subsystems.Flywheel;
//import org.firstinspires.ftc.teamcode.subsystems.Localizer;
//import org.firstinspires.ftc.teamcode.subsystems.Turret;
//
//import com.pedropathing.math.Vector;
//public class Shooter {
//    final double TICKS_PER_REV = 28.0;
//    final double WHEEL_DIAMETER_INCHES = 3.78;
//    final double HOODED_MULTIPLIER = 2.0;
//    double COMPRESSION_TUNING_FACTOR = 1.15;
//    public static Pose GOAL_POS_RED = new Pose(138, 138);
//    public static Pose GOAL_POS_BLUE = GOAL_POS_RED.mirror();
//    public static double SCORE_HEIGHT = 26;
//    public static double SCORE_ANGLE = Math.toRadians(-30);
//    public static double PASS_THROUGH_POINT_RADIUS = 5;
//    public static double MIN_HOOD_ANGLE = 35;
//    public static double MAX_HOOD_ANGLE = 60;
//    double hoodPos, turretAngle, flywheelTicks;
//    Flywheel flywheel = new Flywheel();
//    public void calculateShootVectorAndUpdateTurret(double robotHeading, Localizer localizer) {
//
//        // 1. Get current position from your localizer
//        double xPos = localizer.getXPos();
//        double yPos = localizer.getYPos();
//
//        // 2. Create the Goal Vector (Field-Centric)
//        double xDistance = Turret.BLUE_GOAL_X_POS - xPos;
//        double yDistance = Turret.GOAL_Y_POS - yPos;
//        Vector robotToGoalVector = new Vector(xDistance, yDistance);
//
//        // 3. Get the Velocity Vector from your localizer (Replaces hardware.poseTracker.getVelocity())
//        Vector robotVelocity = localizer.getVelocity();
//
//        // --- START OF TUTORIAL MATH FROM SCREENSHOT ---
//
//        // Calculate the angle difference between movement and the goal
//        double coordinateTheta = robotVelocity.getTheta() - robotToGoalVector.getTheta();
//
//        // Get parallel and perpendicular velocity components for compensation
//        double parallelComponent = -Math.cos(coordinateTheta) * robotVelocity.getMagnitude();
//        double perpendicularComponent = Math.sin(coordinateTheta) * robotVelocity.getMagnitude();
//
//        // Physics constants from your screenshot
//        double g = 32.174 * 12; // Gravity in inches/s^2
//        double x = robotToGoalVector.getMagnitude() - ShooterConstants.PASS_THROUGH_POINT_RADIUS;
//        double y = ShooterConstants.SCORE_HEIGHT;
//        double a = ShooterConstants.SCORE_ANGLE;
//
//        // Initial launch component calculations
//        double hoodAngle = Math.atan(2 * y / x - Math.tan(a));
//        // Clip the hood angle to your mechanical limits
//        hoodAngle = Range.clip(hoodAngle, ShooterConstants.MIN_HOOD_ANGLE, ShooterConstants.MAX_HOOD_ANGLE);
//
//        double flywheelSpeed = Math.sqrt(g * x * x / (2 * Math.pow(Math.cos(hoodAngle), 2) * (x * Math.tan(hoodAngle) - y)));
//
//        // Velocity compensation variables (The 'Moving Shot' logic)
//        double vz = flywheelSpeed * Math.sin(hoodAngle);
//        double time = x / (flywheelSpeed * Math.cos(hoodAngle));
//        double ivr = x / time + parallelComponent;
//        double nvr = Math.sqrt(ivr * ivr + perpendicularComponent * perpendicularComponent);
//        double ndr = nvr * time;
//
//        // Recalculate final launch components based on motion
//        hoodAngle = Math.atan(vz / nvr);
//        hoodAngle = Range.clip(hoodAngle, ShooterConstants.MIN_HOOD_ANGLE, ShooterConstants.MAX_HOOD_ANGLE);
//
//        flywheelSpeed = Math.sqrt(g * ndr * ndr / (2 * Math.pow(Math.cos(hoodAngle), 2) * (ndr * Math.tan(hoodAngle) - y)));
//        flywheelTicks = shooterMISC.velocityToTicks(flywheelSpeed);
//        double turretVelCompOffset = Math.atan(perpendicularComponent / ivr);
//        turretAngle = Math.toDegrees(robotHeading - robotToGoalVector.getTheta() + turretVelCompOffset);
//        turretAngle = turret.correctAngle(turretAngle);
//        hoodPos = shooterConstants.angleToPos(hoodAngle);
//    }
//    public double getHoodPos (){
//        return hoodPos;
//    }
//    public double getTurretAngle () {
//        return turretAngle;
//    }
//    public double getFlywheelTicks () {
//        return flywheelTicks;
//    }
//    public double angleToPos (double angle) {
//        double pos = (angle - 35) * 1/25;
//        //remember to set servo limits to 0, 0.076
//        return pos;
//    }
//    public double velocityToTicks (double velocity) {
//        double circumference = WHEEL_DIAMETER_INCHES * Math.PI;
//
//        double baseRotationsPerSec = velocity / circumference;
//
//        double hoodedRotationsPerSec = baseRotationsPerSec * HOODED_MULTIPLIER;
//
//        double actualRotationsPerSec = hoodedRotationsPerSec * COMPRESSION_TUNING_FACTOR;
//
//        double ticks = actualRotationsPerSec * TICKS_PER_REV;
//
//        return ticks;
//    }
//}