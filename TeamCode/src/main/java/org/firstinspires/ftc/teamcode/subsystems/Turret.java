    package org.firstinspires.ftc.teamcode.subsystems;

    import com.qualcomm.robotcore.hardware.DcMotor;
    import com.qualcomm.robotcore.hardware.DcMotorEx;
    import com.qualcomm.robotcore.hardware.DcMotorSimple;
    import com.qualcomm.robotcore.hardware.HardwareMap;
    import com.qualcomm.robotcore.util.ElapsedTime;

    import org.firstinspires.ftc.robotcore.external.Telemetry;
    import org.firstinspires.ftc.robotcore.external.navigation.AngleUnit;

    public class Turret {
        private DcMotorEx turret;
        private final ElapsedTime initTimer = new ElapsedTime();
        //Goal Position:
        public final static double BLUE_GOAL_X_POS = 0;
        public final static double RED_GOAL_X_POS = 144;
        public final static double GOAL_Y_POS = 144;
        //Ticks, Gear Ratios & TICKS_PER_DEGREE
        private final static double TICKS_PER_REV = 537.7;
        private final static double GEAR_RATIO = 4;
        public final static double TICKS_PER_DEGREE = (TICKS_PER_REV * GEAR_RATIO) / 360.0;
        //Checking Change Logic
        private int lastTargetTicks = 0;
        private final static double BLUE_TURRET_OFFSET = 0.0;
        private final static double RED_TURRET_OFFSET = 0.0;
        private final static double TURRET_FORWARD_OFFSET = 0.455;
        private boolean set, done = false;

        public void init(HardwareMap hwMap) {
            turret = hwMap.get(DcMotorEx.class, "turret");
            turret.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
            turret.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);
            turret.setDirection(DcMotorSimple.Direction.REVERSE);

            initTimer.reset();
        }
        public void init_loop(Telemetry telemetry) {
            if (initTimer.seconds() < 2) {
                turret.setPower(0.3);
            }
            else {
                turret.setPower(0.0);
                done = true;
            }
            if (!set && done) {
                turret.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);
                turret.setTargetPosition(0); // Good practice before switching to RUN_TO_POSITION
                turret.setDirection(DcMotorEx.Direction.FORWARD);
                turret.setMode(DcMotor.RunMode.RUN_TO_POSITION);
                turret.setPower(1.0);
                set = true;
                telemetry.addLine("Turret Initialized!");
            }
        }
        public void trackBlue(double xPos, double yPos, double currentHeading) {
            // Convert heading to radians for Java's trig functions
            double headingRad = Math.toRadians(currentHeading);

            // Translate robot center coordinates to turret center coordinates
            double turretX = xPos + (TURRET_FORWARD_OFFSET * Math.cos(headingRad));
            double turretY = yPos + (TURRET_FORWARD_OFFSET * Math.sin(headingRad));

            // Calculate distance from the TURRET to the goal
            double xDistance = BLUE_GOAL_X_POS - turretX;
            double yDistance = GOAL_Y_POS - turretY;

            track(xDistance, yDistance, currentHeading, BLUE_TURRET_OFFSET);
        }

        public void trackRed(double xPos, double yPos, double currentHeading) {
            // Convert heading to radians for Java's trig functions
            double headingRad = Math.toRadians(currentHeading);

            // Translate robot center coordinates to turret center coordinates
            double turretX = xPos + (TURRET_FORWARD_OFFSET * Math.cos(headingRad));
            double turretY = yPos + (TURRET_FORWARD_OFFSET * Math.sin(headingRad));

            // Calculate distance from the TURRET to the goal
            double xDistance = RED_GOAL_X_POS - turretX;
            double yDistance = GOAL_Y_POS - turretY;

            track(xDistance, yDistance, currentHeading, RED_TURRET_OFFSET);
        }
        private void track (double xDistance, double yDistance, double currentHeading,double offset) {
            double globalTargetAngle = Math.toDegrees(Math.atan2(yDistance, xDistance));
            double turretTargetAngle = AngleUnit.normalizeDegrees(globalTargetAngle-currentHeading);

            turretTargetAngle += offset;

            turretTargetAngle = correctAngle(turretTargetAngle);
            int targetTicks = (int) Math.round(turretTargetAngle * TICKS_PER_DEGREE);
            if (Math.abs(targetTicks - lastTargetTicks) >= 1) {
                turret.setTargetPosition(targetTicks);
            }
            lastTargetTicks = targetTicks;
        }
        public double correctAngle (double turretTargetAngle) {
            turretTargetAngle = AngleUnit.normalizeDegrees(turretTargetAngle);
            if (turretTargetAngle < 0 && turretTargetAngle > -90) {
                return 0;
            }
            // If the math wants us to go past 180 (the left stop), stay at 180
            if (turretTargetAngle > 180 || turretTargetAngle < -90) {
                return 180;
            }

            return turretTargetAngle;
        }
        public void setTurret (double angle) {
            turret.setVelocity(angle*TICKS_PER_DEGREE);
        }

    }
