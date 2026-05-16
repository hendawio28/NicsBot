package org.firstinspires.ftc.teamcode.Tests;

import com.pedropathing.geometry.Pose;
import com.pedropathing.math.Vector;
import com.qualcomm.robotcore.util.Range;

import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;
import org.firstinspires.ftc.teamcode.subsystems.Localizer;

public class Shooter {
    // --- Hardware Constants ---
    final double TICKS_PER_REV = 28.0;
    final double WHEEL_DIAMETER_INCHES = 3.78;
    final double HOODED_MULTIPLIER = 2.0;
    double COMPRESSION_TUNING_FACTOR = 1.15;

    // --- Field & Target Constants ---
    public static Pose GOAL_POS_RED = new Pose(138, 138);
    public static Pose GOAL_POS_BLUE = GOAL_POS_RED.mirror();

    // MATHEMATICAL FIX: Y is height ABOVE the shooter, not the floor!
    public static double SCORE_HEIGHT = 26.0;
    public static double SHOOTER_HEIGHT_INCHES = 12.0; // CHANGE THIS to your physical robot!

    public static double SCORE_ANGLE = Math.toRadians(-30);
    public static double PASS_THROUGH_POINT_RADIUS = 5;

    // --- Hood & Turret Limits ---
    public static double MIN_HOOD_ANGLE = 35; // a1
    public static double MAX_HOOD_ANGLE = 45; // a2
    public static double MIN_SERVO_POS = 0.0; // s1
    public static double MAX_SERVO_POS = 1.0; // s2

    // --- Turret Hardware Configuration ---
    // In PedroPathing, 0 is Forward, -90 is Right.
    public static double TURRET_HARD_STOP_OFFSET_DEG = -90.0;

    // --- Calculated Outputs ---
    double hoodPos;
    double turretAngle;
    double flywheelTicks;
    boolean targetInBounds;

    public void calculateShootVectorAndUpdateTurret(Localizer localizer) {

        double xPos = localizer.getXPos();
        double yPos = localizer.getYPos();
        double robotHeadingRad = Math.toRadians(localizer.getHeading());

        // Create the Goal Vector (Field-Centric)
        double xDistance = GOAL_POS_BLUE.getX() - xPos;
        double yDistance = GOAL_POS_BLUE.getY() - yPos;
        Vector robotToGoalVector = new Vector(xDistance, yDistance);
        Vector robotVelocity = localizer.getVelocity();

        // --- A. Calculate Initial Ball Launch Angle and Velocity ---

        double coordinateTheta = robotVelocity.getTheta() - robotToGoalVector.getTheta();

        // Radial & Tangential compensation
        double vrr = -Math.cos(coordinateTheta) * robotVelocity.getMagnitude();
        double vrt = Math.sin(coordinateTheta) * robotVelocity.getMagnitude();

        double g = 386.1; // Gravity in in/s^2

        // Setup trajectory limits
        double x = Math.max(robotToGoalVector.getMagnitude() - PASS_THROUGH_POINT_RADIUS, 1.0);
        double y = SCORE_HEIGHT - SHOOTER_HEIGHT_INCHES; // CRITICAL: Height relative to shooter
        double targetAngle = SCORE_ANGLE;

        // Initial launch angle (Formula 5 from PDF)
        double hoodAngleRad = Math.atan((2 * y / x) - Math.tan(targetAngle));

        double minAngleRad = Math.toRadians(MIN_HOOD_ANGLE);
        double maxAngleRad = Math.toRadians(MAX_HOOD_ANGLE);
        hoodAngleRad = Range.clip(hoodAngleRad, minAngleRad, maxAngleRad);

        // Initial launch speed (Formula 6 from PDF)
        double denominator1 = (x * Math.tan(hoodAngleRad) - y);
        double flywheelSpeed = 0;
        if (denominator1 > 0) {
            flywheelSpeed = Math.sqrt((g * x * x) / (2 * Math.pow(Math.cos(hoodAngleRad), 2) * denominator1));
        }

        // --- B. Velocity Compensation ---

        double vy = flywheelSpeed * Math.sin(hoodAngleRad);
        double vx = flywheelSpeed * Math.cos(hoodAngleRad);
        double time = x / vx;

        double vxCompensated = vx + vrr;
        double vxNew = Math.hypot(vxCompensated, vrt); // Safe pythagorean hypotenuse
        double xNew = vxNew * time;

        // New launch angle (Formula B.5 from PDF)
        hoodAngleRad = Math.atan2(vy, vxNew);
        hoodAngleRad = Range.clip(hoodAngleRad, minAngleRad, maxAngleRad);

        // New launch speed based on new X distance (Formula B.6 from PDF)
        double denominator2 = (xNew * Math.tan(hoodAngleRad) - y);
        if (denominator2 > 0) {
            flywheelSpeed = Math.sqrt((g * xNew * xNew) / (2 * Math.pow(Math.cos(hoodAngleRad), 2) * denominator2));
        } else {
            flywheelSpeed = 0; // Physically impossible trajectory (avoids NaN crash)
        }

        // --- C. Convert to Hardware Commands ---

        flywheelTicks = velocityToTicks(flywheelSpeed);

        // Fixes typo in PDF: Must use atan2, not tan!
        double turretVelCompOffsetRad = Math.atan2(vrt, vxCompensated);

        // Calculate absolute target relative to field, then subtract robot heading
        double absoluteTargetHeading = robotToGoalVector.getTheta() - turretVelCompOffsetRad;

        double relativeToRobotDeg = Math.toDegrees(absoluteTargetHeading - robotHeadingRad);
        relativeToRobotDeg = AngleUnit.normalizeDegrees(relativeToRobotDeg);

        // Map to the 180-degree physical sweep (0 degrees = Right Hard Stop)
        turretAngle = relativeToRobotDeg - TURRET_HARD_STOP_OFFSET_DEG;

        // Hardware safety clamp: Prevent commanding past the 180-degree sweep
        if (turretAngle < 0 || turretAngle > 180) {
            targetInBounds = false;
            turretAngle = Range.clip(turretAngle, 0, 180);
            flywheelTicks = 0; // Don't spin up if we physically can't aim at it
        } else {
            targetInBounds = true;
        }

        // Map hood angle to servo (Formula C.1 from PDF)
        hoodPos = angleToPos(Math.toDegrees(hoodAngleRad));
    }

    // --- GETTERS ---
    public double getHoodPos() { return hoodPos; }
    public double getTurretAngle() { return turretAngle; }
    public double getFlywheelTicks() { return flywheelTicks; }
    public boolean isTargetInBounds() { return targetInBounds; }

    // --- HARDWARE CONVERSION HELPERS ---

    public double angleToPos(double alpha) {
        double slope = (MIN_SERVO_POS - MAX_SERVO_POS) / (MIN_HOOD_ANGLE - MAX_HOOD_ANGLE);
        return (slope * (alpha - MIN_HOOD_ANGLE)) + MIN_SERVO_POS;
    }

    public double velocityToTicks(double velocity) {
        if (velocity <= 0) return 0;

        double circumference = WHEEL_DIAMETER_INCHES * Math.PI;
        double baseRotationsPerSec = velocity / circumference;
        double hoodedRotationsPerSec = baseRotationsPerSec * HOODED_MULTIPLIER;
        double actualRotationsPerSec = hoodedRotationsPerSec * COMPRESSION_TUNING_FACTOR;
        return actualRotationsPerSec * TICKS_PER_REV;
    }
}