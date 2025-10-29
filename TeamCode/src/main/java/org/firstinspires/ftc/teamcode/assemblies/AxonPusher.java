package org.firstinspires.ftc.teamcode.assemblies;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.digitalchickenlabs.OctoQuad;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.teamcode.libs.teamUtil;

import java.util.Arrays;
import java.util.concurrent.atomic.AtomicBoolean;

@Config
public class AxonPusher{
    public CRServo axon;
    public OctoQuadFWv3 octoquad;
    public OctoQuadFWv3.EncoderDataBlock encoders;
    public final int ODO_PUSHER = 3; // Intake Slider octoquad port

    AtomicBoolean moving = new AtomicBoolean(false);
    public AtomicBoolean timedOut = new AtomicBoolean(false);
    public static boolean details = false;
    public static float POWER_ADJUSTEMENT = -.01f;
    public static float RTP_MAX_VELOCITY = .5f;
    public static int RTP_SLOW_THRESHOLD = 1500; // TODO Recalibrate
    public static float RTP_SLOW_VELOCITY = .14f; // TODO Recalibrate
    public boolean CALIBRATED = false;

    //292 slider degrees to 10 cm
    // 1 mm = 51.2727 tics
    //LEFT IS NEGATIVE!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!!



    public static int axonRotations = 0; // The number of full rotations postive or negative the servo has traveled from its center range

    public void init(HardwareMap hardwareMap, String servoName){
        teamUtil.log("Init AxonSlider");
        axon = hardwareMap.crservo.get(servoName);
        octoquad = hardwareMap.get(OctoQuadFWv3.class, "octoquad");
        octoquad.setSingleEncoderDirection(ODO_PUSHER,  OctoQuadFWv3.EncoderDirection.FORWARD); // RIGHT IS POSITIVE, LEFT IS NEGATIVE
        //axonPotentiometer = hardwareMap.analogInput.get(sensorName); Not USED ANYMORE
        axonRotations=0; // presumes we are in the middle rotation.  Run Calibrate to be sure.
    }

    public void calibrateEncoder(float power) {
        // TODO: make calibration based on the potentiometer to calibrate the octoquad position
        CALIBRATED = false;
        teamUtil.log("Calibrating Pusher");
        CALIBRATED = true;
    }
    /*
    // Flipper must be in a safe position for travel to the far right side
    // Leaves the slider on the far left
    public void calibrate (float power, int rotations) {
        CALIBRATED = false;
        teamUtil.log("Calibrating Intake Slider");
        axon.setPower(power);
        int lastPosition = getPosition();
        teamUtil.pause(250);
        while ((int)getPosition() != lastPosition) {
            lastPosition = getPosition();
            if (details) teamUtil.log("Calibrate Intake: Slider: " + getPosition());
            teamUtil.pause(50);
        }
        axon.setPower(0);
        teamUtil.log("Calibrate Intake Slider Done");

        axonRotations = rotations;
        teamUtil.log("Calibrate POS: " + getPosition());
        teamUtil.pause(250);
        lastDegrees360 = getDegrees360();

        RIGHT_LIMIT = getPosition();
        LEFT_LIMIT = getPosition()+ LEFT_LIMIT_OFFSET;
        SLIDER_READY = getPosition()+ SLIDER_READY_OFFSET;
        SLIDER_UNLOAD = getPosition()+ SLIDER_UNLOAD_OFFSET;
        teamUtil.log("RIGHT LIMIT: " + RIGHT_LIMIT);
        teamUtil.log("LEFT LIMIT: " + LEFT_LIMIT);
        teamUtil.log("SLIDER READY: " + SLIDER_READY);
        teamUtil.log("SLIDER UNLOAD: " + SLIDER_UNLOAD);

        CALIBRATED = true;
    }

     */


    public int getPositionEncoder() {
        return encoders.positions[ODO_PUSHER];
    }

    // The servo is a bit stronger in one direction than the other.  This is evident at very low speeds
    // Call this method to adjust for this
    public void setAdjustedPower(float power){
        if (Math.abs(power-0) < .001f) {
            axon.setPower(0); // don't adjust 0 to non-zero
        } else {
            axon.setPower(power+POWER_ADJUSTEMENT);
        }
    }

    public void setPower(double power){ //-.5 to .5
        axon.setPower(power);
    }


    private void runToTargetEncoder (double target, float velocity, long timeOut) {
        moving.set(true);
        teamUtil.log("Slider runToTargetEncoder: " + (int)target + " at power: "+ velocity);
        long timeoutTime = System.currentTimeMillis()+timeOut;

        double ticsFromTarget = target-getPositionEncoder();
        double lastTicsFromTarget = ticsFromTarget;
        double initialTicsFromTarget = ticsFromTarget;
        setAdjustedPower(ticsFromTarget > 0 ? velocity * -1 : velocity); // start moving
        while (teamUtil.keepGoing(timeoutTime) &&
                initialTicsFromTarget < 0) {// while we haven't yet reached the drift threshold

            if (details)
                teamUtil.log("Tics from Target: " + ticsFromTarget + " Power: " + velocity);
            lastTicsFromTarget = ticsFromTarget;
            // teamutil.pause(10);
            ticsFromTarget = target - getPositionEncoder();

        }
        axon.setPower(0);
        moving.set(false);
        if (System.currentTimeMillis() > timeoutTime) {
            timedOut.set(true);
            teamUtil.log("Slider runToTarget TIMED OUT: " + (int)getPositionEncoder());
        } else {
            teamUtil.log("Slider runToTarget Finished at : " + (int)getPositionEncoder());
        }
    }


    // Run the servo to the specified position as quickly as possible
    // This method returns when the servo is in the new position
    public void runToEncoderPosition (double target, float velocity, long timeOut) {
        timedOut.set(false);
        moving.set(true);
        teamUtil.log("Slider Run to Position Target Encoder : " + (int)target);
        runToTargetEncoder(target, velocity, timeOut);
        teamUtil.log("Slider Run to Position Target Encoder Finished at : " + (int)getPositionEncoder());
    }


    public void runToEncoderPositionNoWait(double target, float velocity, long timeOutTime) {
        if (moving.get()) { // Slider is already running in another thread
            teamUtil.log("WARNING: Attempt to AxonSlider.RunToPosition while slider is moving--ignored");
            return;
        } else {
            moving.set(true);
            teamUtil.log("Launching Thread to AxonSlider.RunToPosition");
            Thread thread = new Thread(new Runnable() {
                @Override
                public void run() {
                    runToEncoderPosition(target, velocity, timeOutTime);
                }
            });
            thread.start();
        }
    }

    public void loop(){
        encoders = octoquad.readAllEncoderData();
    }
}
