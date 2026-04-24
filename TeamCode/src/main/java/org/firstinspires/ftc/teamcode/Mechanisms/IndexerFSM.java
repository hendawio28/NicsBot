package org.firstinspires.ftc.teamcode.Mechanisms;

import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.util.ElapsedTime;

public class IndexerFSM {
    private DcMotor indexerMotor;

    private final double TICKS_PER_REV = 537.7;
    private int currentSlot, targetPosition = 0;
    private IndexerState currentState = IndexerState.IDLE;
    Kicker kicker = new Kicker();
    // The internal stopwatch for the kicker delays

    private ElapsedTime timer = new ElapsedTime();
    public enum IndexerState {
        IDLE,
        GO_TO_INTAKE,
        GO_TO_SHOOT,
        WAITING_FOR_INDEXER, // Waits for the motor to reach the slot
        KICKING,             // Waits for the servo to push out
        RETRACTING           // Waits for the servo to pull back in
    }

    public void init(HardwareMap hwMap) {
        indexerMotor = hwMap.get(DcMotor.class, "Indexer");

        indexerMotor.setMode(DcMotor.RunMode.STOP_AND_RESET_ENCODER);

        indexerMotor.setTargetPosition(targetPosition);

        indexerMotor.setMode(DcMotor.RunMode.RUN_TO_POSITION);

        indexerMotor.setZeroPowerBehavior(DcMotor.ZeroPowerBehavior.BRAKE);

        kicker.init(hwMap);

    }
    public void setTargetState(IndexerState newState) {
        if (currentState == IndexerState.IDLE) {
            this.currentState = newState;
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
                targetPosition = (int) Math.round((TICKS_PER_REV / 6.0) * currentSlot);
                indexerMotor.setTargetPosition(targetPosition);
                indexerMotor.setPower(1.0);
                currentState = IndexerState.IDLE; // Reset so it only spins once!
                break;
            case GO_TO_SHOOT:
                if (isEven) {
                    currentSlot += 1; // Jump from intake to shoot
                } else {
                    currentSlot += 2; // Jump to next shoot slot
                }
                targetPosition = (int) Math.round((TICKS_PER_REV / 6.0) * currentSlot);
                indexerMotor.setTargetPosition(targetPosition);
                indexerMotor.setPower(1.0);
                currentState = IndexerState.WAITING_FOR_INDEXER; // Reset so it only spins once!
                break;
            case WAITING_FOR_INDEXER:
                int currentPos = indexerMotor.getCurrentPosition();

                // Calculate how far away we are from the target
                int distanceLeft = Math.abs(targetPosition - currentPos);
                if (distanceLeft < 10) {
                    kicker.kick(1.0);
                    timer.reset();
                    currentState = IndexerState.KICKING;
                }
                break;
            case KICKING:
                if (timer.seconds() > 0.3) {
                    kicker.kick(0.0);
                    timer.reset();
                    currentState = IndexerState.RETRACTING;
                }
                break;
            case RETRACTING:
                if (timer.seconds() > 0.3) {
                    currentState = IndexerState.IDLE;
                }
                break;
        }
    }
    // Helpful for Telemetry!
    public String getStateName() {
        return currentState.toString();
    }

    public int getCurrentSlot() {
        return currentSlot;
    }
}

