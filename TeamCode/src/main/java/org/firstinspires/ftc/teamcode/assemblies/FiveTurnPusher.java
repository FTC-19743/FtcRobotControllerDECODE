package org.firstinspires.ftc.teamcode.assemblies;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.libs.OctoQuadFWv3;
import org.firstinspires.ftc.teamcode.libs.teamUtil;

import java.util.concurrent.atomic.AtomicBoolean;

@Config
public class FiveTurnPusher {
    HardwareMap hardwareMap;
    Telemetry telemetry;
    public Servo servo;

    public final int ODO_PUSHER = 3; // Pusher encoder octoquad port
    public static float[] READY_POS = {.91f,.8f, .7f, .587f, .47f, .355f, .24f, .125f, .02f};
    public static int REV_ENCODER_TICS_PER_REVOLUTION = 8192;
    public final int PUSHER_PADDLES = 2;
    public int ENCODER_LOAD_POSITION_1 = 0;
    public static float PUSH_SLOW_VELOCITY = .5f; // NO slow velocity for now
    public static int PUSH_N_PAUSE = 300;

    AtomicBoolean moving = new AtomicBoolean(false);
    public AtomicBoolean timedOut = new AtomicBoolean(false);
    public static boolean details = false;
    public static float RTP_MAX_VELOCITY = .5f;
    public boolean CALIBRATED = false;
    public static long OQ_READ_TIMEOUT = 30;


    public FiveTurnPusher() {
        teamUtil.log("Constructing 5TurnPusher");
        hardwareMap = teamUtil.theOpMode.hardwareMap;
        telemetry = teamUtil.theOpMode.telemetry;
    }
    public void initialize(){
        teamUtil.log("Init 5Turn Pusher");
        servo = hardwareMap.servo.get("pusher");
        teamUtil.robot.oq.setSingleEncoderDirection(ODO_PUSHER,  OctoQuadFWv3.EncoderDirection.FORWARD);
    }

    public void reset(boolean waitForFinish) {
        servo.setPosition(READY_POS[0]);
        if (waitForFinish) {
            long timeOutTime = System.currentTimeMillis() + 3000;
            teamUtil.pause(200);
            getPositionEncoder();
            while (lastValidEncoderVelocity > 0 && teamUtil.keepGoing(timeOutTime)) {
                teamUtil.pause(20);
            }
        }
    }

    // Reset and note starting encoder position
    public void calibrate() {
        timedOut.set(false);
        CALIBRATED = false;
        teamUtil.log(String.format("Calibrating 5Turn Pusher. Enc: %d ", getPositionEncoder()));
        reset(true);
        ENCODER_LOAD_POSITION_1 = getPositionEncoder();
        teamUtil.log("calibrate Finished");
        CALIBRATED = true;
    }

    public void calibrateNoWait() {
        if (moving.get()) { // Pusher is already running in another thread
            teamUtil.log("WARNING: Attempt to FiveTurnPusher.calibrate while Pusher is moving--ignored");
            return;
        } else {
            teamUtil.log("Launching Thread to FiveTurnPusher.calibrate");
            Thread thread = new Thread(new Runnable() {
                @Override
                public void run() {
                    calibrate();
                }
            });
            thread.start();
        }
    }


    int lastValidEncoder;
    int lastValidEncoderVelocity;
    public int getPositionEncoder() {
        OctoQuadFWv3.EncoderDataBlock encoders;
        encoders = teamUtil.robot.oq.readAllEncoderData();
        long timeoutTime = System.currentTimeMillis() + OQ_READ_TIMEOUT;
        while(!encoders.crcOk && teamUtil.keepGoing(timeoutTime)){
            encoders = teamUtil.robot.oq.readAllEncoderData();
            teamUtil.log("ERROR ------------- Pusher OQ returned bad CRC");
        }
        if(encoders.crcOk) {
            lastValidEncoder = encoders.positions[ODO_PUSHER];
            lastValidEncoderVelocity = encoders.velocities[ODO_PUSHER];
            return encoders.positions[ODO_PUSHER];
        }
        teamUtil.log("ERROR ------------- Timed out waiting for good CRC from Pusher OQ");
        return lastValidEncoder;
    }




    public void outputTelemetry(){
        telemetry.addLine("Pusher octoquad: "+getPositionEncoder() + " Pusher Pos: "+servo.getPosition());
    }

    public int getCurrentPushPosition () {
        int nearestIndex = 0;
        // Initialize minDiff with the absolute difference of the first element
        double minDiff = Math.abs(servo.getPosition() - READY_POS[0]);
        for (int i = 1; i < READY_POS.length; i++) {
            double currentDiff = Math.abs(servo.getPosition() - READY_POS[i]);
            if (currentDiff < minDiff) {
                minDiff = currentDiff;
                nearestIndex = i;
            }
        }
        return nearestIndex;
    }

    public int getNextPushPosition() {
        int nextIndex = getCurrentPushPosition() + 1;
        if (nextIndex >= READY_POS.length) {
            teamUtil.log("WARNING: getNextPushPosition called when no next position available");
            return 0;
        } else {
            return nextIndex;
        }
    }

    public int getPreviousPushPosition() {
        int previousIndex = getCurrentPushPosition() - 1;
        if (previousIndex < 0) {
            teamUtil.log("WARNING: getPreviousPushPosition called when no previous position available");
            return -1;
        } else {
            return previousIndex;
        }
    }

    // Run the servo to the next position if there is one
    // This method returns when the servo is in the new position
    public static int ENCODER_DONE_THRESHOLD = 1500;
    public void push1 (long timeOut) {
        teamUtil.log("push1");
        long startTime = System.currentTimeMillis();
        long timeOutTime = startTime + timeOut;
        timedOut.set(false);
        moving.set(true);
        int nextPos = getNextPushPosition();
        if (nextPos == 0) {
            teamUtil.log("WARNING: Push1 called when no next position available. Ignoring");
            return;
        }
        int nextEncoder = getPositionEncoder()+REV_ENCODER_TICS_PER_REVOLUTION/PUSHER_PADDLES - ENCODER_DONE_THRESHOLD;
        servo.setPosition(READY_POS[nextPos]);
        teamUtil.pause(100);
        getPositionEncoder();
        while (getPositionEncoder() < nextEncoder && teamUtil.keepGoing(timeOutTime)) {
            teamUtil.pause(20);
            if (details) teamUtil.log("Encoder Position: " + lastValidEncoder + " Velocity: " + lastValidEncoderVelocity);
        }
        // TODO: Detect Stall and either signal or do something about it

        teamUtil.log("push1 Finished in " + (System.currentTimeMillis() - startTime) + " ms at Encoder: " + getPositionEncoder());
    }

    // Run the servo to the previous position if there is one
    // This method returns when the servo is in the new position
    public void reverse1 (long timeOut) {
        teamUtil.log("reverse1");
        long timeOutTime = System.currentTimeMillis() + timeOut;
        timedOut.set(false);
        moving.set(true);
        int nextPos = getPreviousPushPosition();
        if (nextPos == -1) {
            teamUtil.log("WARNING: reverse1 called when no previous position available. Ignoring");
            return;
        }
        servo.setPosition(READY_POS[nextPos]);
        teamUtil.pause(100);
        getPositionEncoder();
        while (Math.abs(lastValidEncoderVelocity) > 0 && teamUtil.keepGoing(timeOutTime)) {
            teamUtil.pause(20);
            getPositionEncoder();
            if (details) teamUtil.log("Encoder Velocity: " + lastValidEncoderVelocity);
        }
        // TODO: Detect Stall and either signal or do something about it

        teamUtil.log("reverse1 Finished at Encoder: " + getPositionEncoder());
    }

    public void pushN(int num, float velocity, long timeout) {
        teamUtil.log("PushN num: " + num);
        moving.set(true);
        for(int i = 0;i < num;i++){// todo: fix timeout to be for all of them instead of individually
            int encoderPosition = getPositionEncoder();
            long timeoutTime = timeout + System.currentTimeMillis();
            push1(1000);
            if(timedOut.get()){
                teamUtil.log("PushN timed out");
                break;
            }
            if(i<num-1){
                teamUtil.pause(PUSH_N_PAUSE);
            }
        }
        moving.set(false);
        teamUtil.log("PushN Finished");
    }

    public void pushNNoWait(int num, float velocity, long timeout) {
        if (moving.get()) { // Pusher is already running in another thread
            teamUtil.log("WARNING: Attempt to AxonPusher.pushNNoWait while Pusher is moving--ignored");
            return;
        } else {
            moving.set(true);
            teamUtil.log("Launching Thread to AxonPusher.pushNNoWait");
            Thread thread = new Thread(new Runnable() {
                @Override
                public void run() {
                    pushN(num, velocity, timeout);
                }
            });
            thread.start();
        }
    }

}
