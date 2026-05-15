package org.firstinspires.ftc.teamcode.subsystems;

import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

public class IndexerFSM2 {
    private DcMotorEx indexerMotor;
    Kicker kicker = new Kicker();
    private final double TICKS_PER_REV = 537.7;
    private final double TICKS_PER_SLOT = TICKS_PER_REV / 6.0;
    private int currentSlot, targetPosition, shotsRemaining = 0;
    private IndexerState currentState = IndexerState.IDLE;

    // The internal stopwatch for the kicker delays

    private ElapsedTime intakeTimer = new ElapsedTime();
    private ElapsedTime kickerTimer = new ElapsedTime();
    private boolean intakeWasOn = false;
    public enum IndexerState {
        IDLE,
        GO_TO_INTAKE,
        GO_TO_SHOOT,
        WAITING_FOR_INDEXER, // Waits for the motor to reach the slot
        KICKING,             // Waits for the servo to push out
        RETRACTING           // Waits for the servo to pull back in
    }

    public void init(HardwareMap hwMap) {
        indexerMotor = hwMap.get(DcMotorEx.class, "Indexer");

        indexerMotor.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);

        indexerMotor.setTargetPosition(targetPosition);

        indexerMotor.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);

        indexerMotor.setZeroPowerBehavior(DcMotorEx.ZeroPowerBehavior.BRAKE);

        indexerMotor.setPower(1.0);

        kicker.init(hwMap);

        intakeTimer.reset();
    }
    public void setTargetState(boolean launchAll, boolean shoot, boolean intake) {
        boolean spinForIntake = false;

        // 1. Internal Edge Detection & Timing for the Intake
        if (intake && !intakeWasOn) {
            spinForIntake = true;
            intakeTimer.reset();
        } else if (intake && intakeTimer.seconds() >= 1.5) {
            spinForIntake = true;
            intakeTimer.reset();
        }
        intakeWasOn = intake; // Remember state for the next loop

        // 2. Safely Assign Target States (Only if we aren't currently busy!)
        if (currentState == IndexerState.IDLE) {
            if (launchAll) {
                shotsRemaining = 3;
                currentState = IndexerState.GO_TO_SHOOT;
            }
            else if (shoot) {
                shotsRemaining = 1;
                currentState = IndexerState.GO_TO_SHOOT;
            }
            else if (spinForIntake) {
                currentState = IndexerState.GO_TO_INTAKE;
            }
        }
    }

    public void update() {
        boolean isEven = (currentSlot % 2 == 0);

        switch (currentState) {
            case IDLE:
                //Only holds posiiton
                break;
            case GO_TO_INTAKE:
                if (isEven) {
                    currentSlot +=2;
                }
                else  {
                    currentSlot+=1;
                }
                rotate();
                currentState = IndexerState.IDLE; // Reset so it only spins once!
                break;
            case GO_TO_SHOOT:
                if (isEven) {
                    currentSlot += 1; // Jump from intake to shoot
                } else {
                    currentSlot += 2; // Jump to next shoot slot
                }
                rotate();
                currentState = IndexerState.WAITING_FOR_INDEXER; // Reset so it only spins once!
                break;
            case WAITING_FOR_INDEXER:
                int currentPos = indexerMotor.getCurrentPosition();

                // Calculate how far away we are from the target
                int distanceLeft = Math.abs(targetPosition - currentPos);
                if (distanceLeft < 10) {
                    kicker.kick(1.0);
                    kickerTimer.reset();
                    currentState = IndexerState.KICKING;
                }
                break;
            case KICKING:
                if (kickerTimer.seconds() > 0.3) {
                    kicker.kick(0.0);
                    kickerTimer.reset();
                    currentState = IndexerState.RETRACTING;
                }
                break;
            case RETRACTING:
                // 1. Wait for the servo to physically pull back
                if (kickerTimer.seconds() > 0.3) {

                    // 2. Subtract one shot from the counter
                    shotsRemaining--;

                    // 3. Decide what to do next based on the counter
                    if (shotsRemaining > 0) {
                        currentState = IndexerState.GO_TO_SHOOT; // Loop back for next ball
                    } else {
                        currentState = IndexerState.IDLE; // All done!
                    }
                }
                break;
        }
    }
    // Helper method to keep code DRY (Don't Repeat Yourself)
    private void rotate() {
        targetPosition = (int) Math.round(TICKS_PER_SLOT * currentSlot);
        indexerMotor.setTargetPosition(targetPosition);
    }

    // Helpful for Telemetry!
    public String getStateName() {
        return currentState.toString();
    }

    public int getCurrentSlot() {
        return currentSlot;
    }
}

