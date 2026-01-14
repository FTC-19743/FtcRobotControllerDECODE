package org.firstinspires.ftc.teamcode.assemblies;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.libs.OctoQuadFWv3;
import org.firstinspires.ftc.teamcode.libs.teamUtil;

import java.util.concurrent.atomic.AtomicBoolean;

@Config
public class AxonPusher {
    HardwareMap hardwareMap;
    Telemetry telemetry;
    public CRServo servo;
    public AnalogInput servoPot;

    public final int ODO_PUSHER = 3; // Pusher encoder octoquad port
    public static int REV_ENCODER_TICS_PER_REVOLUTION = 8192;
    public final double AXON_POT_FULL_REVOLUTION = 3.3f;
    public static double AXON_LOAD_1 = .04;
    public final int PUSHER_PADDLES = 2;
    //public final double AXON_LOAD_2 = AXON_LOAD_1 + AXON_POT_FULL_REVOLUTION/2;
    public int ENCODER_LOAD_POSITION_1 = 0;
    //public int ENCODER_LOAD_POSITION_2 = 0;
    public static long RUN_TO_ENCODER_PAUSE = 10;
    public static double RUN_TO_ENCODER_DRIFT = 800;
    public static double RUN_TO_ENCODER_SLOW = 1000;

    public static float PUSH_SLOW_VELOCITY = .5f; // NO slow velocity for now
    public static int PUSH_N_PAUSE = 300;


    AtomicBoolean moving = new AtomicBoolean(false);
    public AtomicBoolean timedOut = new AtomicBoolean(false);
    public static boolean details = false;
    public static float POWER_ADJUSTMENT = -.01f;
    public static float RTP_MAX_VELOCITY = .5f;
    public boolean CALIBRATED = false;
    public static long OQ_READ_TIMEOUT = 30;


    public AxonPusher() {
        teamUtil.log("Constructing Pusher");
        hardwareMap = teamUtil.theOpMode.hardwareMap;
        telemetry = teamUtil.theOpMode.telemetry;
    }
    public void initialize(){
        teamUtil.log("Init Pusher");
        servo = hardwareMap.crservo.get("pusher");
        servo.setDirection(DcMotorSimple.Direction.FORWARD);
        servoPot = hardwareMap.analogInput.get("pusherPotentiometer");
        teamUtil.robot.oq.setSingleEncoderDirection(ODO_PUSHER,  OctoQuadFWv3.EncoderDirection.FORWARD);
    }

    public void calibrateNoWait() {
        if (moving.get()) { // Pusher is already running in another thread
            teamUtil.log("WARNING: Attempt to AxonPusher.calibrate while Pusher is moving--ignored");
            return;
        } else {
            teamUtil.log("Launching Thread to AxonPusher.calibrate");
            Thread thread = new Thread(new Runnable() {
                @Override
                public void run() {
                    calibrate();
                }
            });
            thread.start();
        }
    }

    // Use the AxonMax potentiometer to calibrate the Rev Through Bore Encoder.
    // WARNING: This relies on the AxonMax NOT being in the weird state where the potentiometer returns bad data near 0 and 3.3
    // TODO: Move the Axon forward until the potentiometer is returning good data?
    public void calibrate() {
        timedOut.set(false);
        CALIBRATED = false;
        double currentPot = servoPot.getVoltage();
        int currentEncoder = getPositionEncoder();
        // Determine portion of 360 degrees that we need to offset from current position to be in a load position
        double offsetRatio = (AXON_LOAD_1-currentPot)/AXON_POT_FULL_REVOLUTION;
        offsetRatio = offsetRatio * -1; // pot decreases moving forward while encoder increases
        teamUtil.log(String.format("Calibrating Pusher. Pot: %.1f Enc: %d Offset: %.2f", currentPot, currentEncoder, offsetRatio));
        // Use the current encoder position to determine the correct encoder position for a launch using the offsetRatio
        ENCODER_LOAD_POSITION_1 = (int) (offsetRatio * REV_ENCODER_TICS_PER_REVOLUTION + currentEncoder);
        //ENCODER_LOAD_POSITION_2 = ENCODER_LOAD_POSITION_1 + REV_ENCODER_TICS_PER_REVOLUTION/2;
        teamUtil.log("Pusher encoder load 1: "+ENCODER_LOAD_POSITION_1);
        CALIBRATED = true;
    }


    int lastValidEncoder;
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
            return encoders.positions[ODO_PUSHER];
        }
        teamUtil.log("ERROR ------------- Timed out waiting for good CRC from Pusher OQ");
        return lastValidEncoder;
    }

    public double getPot(){
        return servoPot.getVoltage();
    }

    public void setPower(double power){ //-.5 to .5
        servo.setPower(power);
    }

    private void runToTargetEncoderInternal(double target, float velocity, long timeOut) {
        moving.set(true);
        if (details) teamUtil.log("Pusher runToTargetEncoder: " + (int)target + " at power: "+ velocity);
        long timeoutTime = System.currentTimeMillis()+timeOut;

        double ticsFromTarget = target - getPositionEncoder(); // position - target
        setPower(velocity); // start moving
        while (teamUtil.keepGoing(timeoutTime) &&  ticsFromTarget > RUN_TO_ENCODER_DRIFT) {// while we haven't yet reached the target
            if (details)
                teamUtil.log("Tics from Target: " + ticsFromTarget + " Power: " + velocity);
            teamUtil.pause(RUN_TO_ENCODER_PAUSE);
            ticsFromTarget = target - getPositionEncoder();
            if (ticsFromTarget < RUN_TO_ENCODER_SLOW) {
                setPower(PUSH_SLOW_VELOCITY);
                velocity = PUSH_SLOW_VELOCITY;
            }
        }
        servo.setPower(0);
        moving.set(false);
        if (System.currentTimeMillis() > timeoutTime) {
            timedOut.set(true);
            teamUtil.log("Pusher runToTarget TIMED OUT: " + (int)getPositionEncoder());
        } else {
            if (details) teamUtil.log("Pusher runToTarget Finished at : " + (int)getPositionEncoder());
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
        if (details) teamUtil.log("Pusher Run to Position Target Encoder : " + (int)target);
        runToTargetEncoderInternal(target, velocity, timeOut);
        if (details) teamUtil.log("Pusher Run to Position Target Encoder Finished at : " + (int)getPositionEncoder());
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
        float pushesFromStart = distanceFromStart/((float)REV_ENCODER_TICS_PER_REVOLUTION/PUSHER_PADDLES);
        return (Math.round(pushesFromStart+1)*(REV_ENCODER_TICS_PER_REVOLUTION/PUSHER_PADDLES)+ENCODER_LOAD_POSITION_1); // TODO: fix if the pusher is just behind the target
    }

    public void pushN(int num, float velocity, long timeout) {
        moving.set(true);
        for(int i = 0;i < num;i++){// todo: fix timeout to be for all of them instead of individually
//            runToEncoderPosition(nextEncoderTarget(getPositionEncoder())-PUSH_FAST_THRESHOLD, PUSH_SLOW_VELOCITY, timeout); // slow velocity to make the ball not hit the roof
            int encoderPosition = getPositionEncoder();
            long timeoutTime = timeout + System.currentTimeMillis();
            runToEncoderPosition(nextEncoderTarget(getPositionEncoder()), velocity, timeout);// consider making velocity proportional to distance
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
