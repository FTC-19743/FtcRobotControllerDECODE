package org.firstinspires.ftc.teamcode.assemblies;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.libs.teamUtil;

import java.util.concurrent.atomic.AtomicBoolean;

@Config
public class AxonPusher{
    HardwareMap hardwareMap;
    Telemetry telemetry;
    public CRServo servo;
    public AnalogInput servoPot;

    public final int ODO_PUSHER = 3; // Pusher encoder octoquad port
    public final int REV_ENCODER_TICS_PER_REVOLUTION = 8192;
    public final double AXON_POT_FULL_REVOLUTION = 3.3f;
    public final double AXON_LOAD_1 = 1.12; // TODO: Calibrate
    public final double AXON_LOAD_2 = AXON_LOAD_1 + AXON_POT_FULL_REVOLUTION/2;
    public int ENCODER_LOAD_POSITION_1 = 0;
    public int ENCODER_LOAD_POSITION_2 = 0;
    public static long RUN_TO_ENCODER_PAUSE = 15;
    public static double RUN_TO_ENCODER_DRIFT = 0;
    public static float PUSH_SLOW_VELOCITY = .1f;
    public static int PUSH_FAST_THRESHOLD = 3072;


    AtomicBoolean moving = new AtomicBoolean(false);
    public AtomicBoolean timedOut = new AtomicBoolean(false);
    public static boolean details = false;
    public static float POWER_ADJUSTMENT = -.01f;
    public static float RTP_MAX_VELOCITY = .5f;
    public boolean CALIBRATED = false;

    public AxonPusher() {
        teamUtil.log("Constructing Pusher");
        hardwareMap = teamUtil.theOpMode.hardwareMap;
        telemetry = teamUtil.theOpMode.telemetry;
    }
    public void initialize(){
        teamUtil.log("Init Pusher");
        servo = hardwareMap.crservo.get("pusher");
        servoPot = hardwareMap.analogInput.get("pusherPotentiometer");
        teamUtil.robot.oq.setSingleEncoderDirection(ODO_PUSHER,  OctoQuadFWv3.EncoderDirection.FORWARD);
    }

    public void calibrate() {
        // TODO: make calibration based on the potentiometer to calibrate the octoquad position
        CALIBRATED = false;
        teamUtil.log("Calibrating Pusher");
        double currentPot = servoPot.getVoltage();
        int currentEncoder = getPositionEncoder();
        ENCODER_LOAD_POSITION_1 = (int) (-(AXON_LOAD_2-currentPot)/AXON_POT_FULL_REVOLUTION * REV_ENCODER_TICS_PER_REVOLUTION + currentEncoder);
        ENCODER_LOAD_POSITION_2 = ENCODER_LOAD_POSITION_1 + REV_ENCODER_TICS_PER_REVOLUTION/2;
        teamUtil.log("Pusher encoder load 1: "+ENCODER_LOAD_POSITION_1+" Pusher encoder load 2: "+ENCODER_LOAD_POSITION_2);
        CALIBRATED = true;
    }

    public int getPositionEncoder() {
        OctoQuadFWv3.EncoderDataBlock encoders;
        encoders = teamUtil.robot.oq.readAllEncoderData();
        return encoders.positions[ODO_PUSHER];
    }

    public double getPot(){
        return servoPot.getVoltage();
    }

    public void setPower(double power){ //-.5 to .5
        servo.setPower(power);
    }

    private void runToTargetEncoder (double target, float velocity, long timeOut) {
        moving.set(true);
        teamUtil.log("Pusher runToTargetEncoder: " + (int)target + " at power: "+ velocity);
        long timeoutTime = System.currentTimeMillis()+timeOut;

        double ticsFromTarget = target - getPositionEncoder();
        setPower(velocity); // start moving
        while (teamUtil.keepGoing(timeoutTime) &&  ticsFromTarget > RUN_TO_ENCODER_DRIFT) {// while we haven't yet reached the target
            if (details)
                teamUtil.log("Tics from Target: " + ticsFromTarget + " Power: " + velocity);
            teamUtil.pause(RUN_TO_ENCODER_PAUSE);
            ticsFromTarget = target - getPositionEncoder();
        }
        servo.setPower(0);
        moving.set(false);
        if (System.currentTimeMillis() > timeoutTime) {
            timedOut.set(true);
            teamUtil.log("Pusher runToTarget TIMED OUT: " + (int)getPositionEncoder());
        } else {
            teamUtil.log("Pusher runToTarget Finished at : " + (int)getPositionEncoder());
        }
    }

    public void outputTelemetry(){
        telemetry.addLine("Pusher octoquad: "+getPositionEncoder()+(" Pusher potentiometer: "+getPot()));
    }

    // Run the servo to the specified position as quickly as possible
    // This method returns when the servo is in the new position
    public void runToEncoderPosition (double target, float velocity, long timeOut) {
        timedOut.set(false);
        moving.set(true);
        teamUtil.log("Pusher Run to Position Target Encoder : " + (int)target);
        runToTargetEncoder(target, velocity, timeOut);
        teamUtil.log("Pusher Run to Position Target Encoder Finished at : " + (int)getPositionEncoder());
    }


    public void runToEncoderPositionNoWait(double target, float velocity, long timeout) {
        if (moving.get()) { // Slider is already running in another thread
            teamUtil.log("WARNING: Attempt to AxonPusher.RunToPosition while Pusher is moving--ignored");
            return;
        } else {
            moving.set(true);
            teamUtil.log("Launching Thread to AxonPusher.RunToPosition");
            Thread thread = new Thread(new Runnable() {
                @Override
                public void run() {
                    runToEncoderPosition(target, velocity, timeout);
                }
            });
            thread.start();
        }
    }

    public int nextEncoderTarget(int currentEncoder){
        float distanceFromStart = currentEncoder - ENCODER_LOAD_POSITION_1;
        float pushesFromStart = distanceFromStart/((float)REV_ENCODER_TICS_PER_REVOLUTION/2);
        return ((int)pushesFromStart+1)*(REV_ENCODER_TICS_PER_REVOLUTION/2)+ENCODER_LOAD_POSITION_1; // TODO: fix if the pusher is just behind the target
    }

    public void pushN(int num, float velocity, long timeout) {
        moving.set(true);
        teamUtil.log("PushN starting");
        for(int i = 0;i < num;i++){// todo: fix timeout to be for all of them instead of individually
            runToEncoderPosition(nextEncoderTarget(getPositionEncoder())-PUSH_FAST_THRESHOLD, PUSH_SLOW_VELOCITY, timeout); // slow velocity to make the ball not hit the roof
            runToEncoderPosition(nextEncoderTarget(getPositionEncoder()), velocity, timeout);// consider making velocity proportional to distance
            if(timedOut.get()){
                teamUtil.log("PushN timed out");
                break;
            }
        }
        moving.set(false);
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
