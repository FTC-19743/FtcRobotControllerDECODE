package org.firstinspires.ftc.teamcode.assemblies;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.libs.teamUtil;

import java.util.concurrent.atomic.AtomicBoolean;

@Config
public class Shooter {
    HardwareMap hardwareMap;
    Telemetry telemetry;
    public DcMotorEx leftFlywheel;
    public DcMotorEx rightFlywheel;
    //public AxonPusher pusher;
    public FiveTurnPusher pusher;
    private Servo aimer;
    private DigitalChannel loadedSensor;
    private Servo leftPusher;
    private Servo rightPusher;

    public static boolean details;

    public static float LEFT_PUSHER_STOW = .38f;
    public static float LEFT_PUSHER_HOLD = .5f;
    public static float LEFT_PUSHER_PUSH = .73f;
    public static float RIGHT_PUSHER_STOW = .66f;
    public static float RIGHT_PUSHER_HOLD = .59f;
    public static float RIGHT_PUSHER_PUSH = .34f;

    public static float AIMER_CALIBRATE = .4f;
    public static float PUSHER_CALIBRATE_PITCH = .57f;
    public static float AIMER_MIN = .3f;
    public static float AIMER_MAX = .54f;

    public static double SHOOTER_FAR_VELOCITY = 1300;
    public static float PUSHER_VELOCITY = .5f;

    //Old flywheel values pidf

    public static double shooterP = 50;
    public static double shooterI = 1;
    public static double shooterD = 0.8;
    public static double shooterF = 0;

    // New PIDF coeffecients using Feed Forward to get faster recovery
    public static double shooterLeftP = 35;
    public static double shooterLeftI = 0;
    public static double shooterLeftD = 0.05;
    public static double shooterLeftF = 13.5;
    public static double shooterRightP = 35;
    public static double shooterRightI = 0;
    public static double shooterRightD = 0.1;
    public static double shooterRightF = 13.5;

    //THESE are the PIDF numbers to get the flywheels from 0 to 900 fast

    public static double shooterStartP = 50;
    public static double shooterStartI = 4.6;
    public static double shooterStartD = 4;
    public static double shooterStartF = 0;




    public static float AIMER_FAR_LONG = .540f; // far from long
    public static float AIMER_CLOSE_LONG = .515f; // closest from long
    public static float AIMER_FAR_CLOSE = .380f; // furthest from the close
    public static float AIMER_CLOSE_CLOSE = .3f; // closest from the close zone

    public static double FAR_LONG_VELOCITY = 1700;
    public static double CLOSE_LONG_VELOCITY = 1800; // 1600?
    public static double FAR_CLOSE_VELOCITY = 1200;
    public static double CLOSE_CLOSE_VELOCITY = 800;
    public static double SHOOT_3_AIM_CHANGE = .01;

    public static double MID_DISTANCE_THRESHOLD = 2667;
    public static double MID_SHORT_DISTANCE_THRESHOLD = 1400;

    public static double VELOCITY_COMMANDED;
    public static double VELOCITY_COMMANDED_THRESHOLD = 30;
    public static double MAX_IDLE_FLYWHEEL_VELOCITY = 800;

    /// /////////////OLD DATA////////////////////////
    // for aimer:
    // close: .000069913x + .218222
    // far: .0000328084x + .415
    // for v:
    // close: 0.349956x + 391+1/9
    // far: 0.131234x + 1300

    // distances:
    // 12.5ft for furthest
    // closest long shot: 10ft
    // closest close is 3ft 10in inches
    // furthest close shot is 91in

    /// ////////////////NEW DATA////////////////////////////
    // for aimer:
    //close:0.000110345x + 0.20069
    // for V:
    // close: 0.334483x+498.96552

    //distances:
    //shortest short: 900 mm
    //longest short: 2350 mm



    public Shooter() {
        teamUtil.log("Constructing Shooter");
        hardwareMap = teamUtil.theOpMode.hardwareMap;
        telemetry = teamUtil.theOpMode.telemetry;
        //pusher = new AxonPusher();
        pusher = new FiveTurnPusher();

    }
    public void setFlywheelCoefficients(double P, double I, double D, double F){
        leftFlywheel.setVelocityPIDFCoefficients(P, I, D, F);
        rightFlywheel.setVelocityPIDFCoefficients(P, I, D, F);
    }

    public void flywheelStartup(){
        setFlywheelCoefficients(shooterStartP, shooterStartI, shooterStartD, shooterStartF);
        teamUtil.log("set shooter PIDF to startup to get to speed");
    }

    public void flywheelNormal(){
        setFlywheelCoefficients(shooterP, shooterI, shooterD, shooterF);
        teamUtil.log("set shooter PIDF to normal to maintain speed");
    }

    public void flywheelEnhanced() {
        leftFlywheel.setVelocityPIDFCoefficients(shooterLeftP, shooterLeftI, shooterLeftD, shooterLeftF);
        rightFlywheel.setVelocityPIDFCoefficients(shooterRightP, shooterRightI, shooterRightD, shooterRightF);
        teamUtil.log("set shooter PIDF to enhanced for faster recovery");
    }

    public void initialize() {
        teamUtil.log("Initializing Shooter");
        leftFlywheel = hardwareMap.get(DcMotorEx.class,"leftflywheel");
        rightFlywheel = hardwareMap.get(DcMotorEx.class,"rightflywheel");
        leftFlywheel.setDirection(DcMotorSimple.Direction.REVERSE);
        flywheelStartup();
        pusher.initialize();
        aimer = hardwareMap.get(Servo.class,"aimer");
        leftPusher = hardwareMap.get(Servo.class,"leftpusher");
        rightPusher = hardwareMap.get(Servo.class,"rightpusher");

        loadedSensor = hardwareMap.get(DigitalChannel.class, "loaded");
        loadedSensor.setMode(DigitalChannel.Mode.INPUT);

        teamUtil.log("Shooter Initialized");
    }




    public void calibrate(){
        aimer.setPosition(AIMER_CALIBRATE);
        teamUtil.pause (500); // wait for right pitch before moving pusher
        //pusher.setPower(0);
        //calibratePusherV2(); // New stall based calibration
        pusher.calibrate(); // Old calibration for pusher using AxonMax potentiometer
        sidePushersStow();
        // pushOne(); // Not needed with FiveTurnPusher
    }
    public void outputTelemetry(){
        telemetry.addLine("Loaded: "+loadedSensor.getState());
        telemetry.addLine("Left velocity: "+leftFlywheel.getVelocity() +" Right velocity: "+rightFlywheel.getVelocity());
        telemetry.addLine("Aimer position: "+aimer.getPosition());
        pusher.outputTelemetry();
    }
    public void setShootSpeed(double velocity){
        leftFlywheel.setVelocity(velocity);
        rightFlywheel.setVelocity(velocity);
        VELOCITY_COMMANDED = velocity;
    }
    public void stopShooter(){
        leftFlywheel.setVelocity(0);
        rightFlywheel.setVelocity(0);
    }

    public boolean isLoaded() {
        return loadedSensor.getState();
    }

    public void aim (double pos) {
        if (pos<AIMER_MIN) pos = AIMER_MIN;
        if (pos>AIMER_MAX) pos = AIMER_MAX;
        aimer.setPosition(pos);
    }
    public double currentAim () {
        return aimer.getPosition();
    }

    public void pushOneBackwards(long pause){
        pusher.reverse1(1000);
        /*
        pusher.setPower(-1);
        teamUtil.pause(pause);
        pusher.setPower(0);

         */
    }

    public void pushOne(){
        pusher.pushN(1, AxonPusher.RTP_MAX_VELOCITY, 2500);
    }

    public void pushOneNoWait(){
            if (pusher.moving.get()) { // Pusher is already running in another thread
                teamUtil.log("WARNING: Attempt to AxonPusher.pushNNoWait while Pusher is moving--ignored");
                return;
            } else {
                pusher.moving.set(true);
                teamUtil.log("Launching Thread to AxonPusher.pushNNoWait");
                Thread thread = new Thread(new Runnable() {
                    @Override
                    public void run() {
                        pushOne();
                    }
                });
                thread.start();
            }
    }

    public static long SF_SHOT_PAUSE = 100;
    public static long SF_LEFT_PUSH_PAUSE = 200;
    public static long SF_RIGHT_PUSH_PAUSE = 200;
    public static long SF_LEFT_PUSH_PAUSE_NEAR = 200;
    public static long SF_RIGHT_PUSH_PAUSE_NEAR = 200;
    public static long SF_LEFT_PUSH_PAUSE_FAR = 300;
    public static long SF_RIGHT_PUSH_PAUSE_FAR = 300;
    public static long SF_LAST_SHOT_PAUSE = 100;
    public static long SF_LAST_PADDLE_PAUSE = 100;

    public AtomicBoolean superFastShooting = new AtomicBoolean(false);
    // Assumes 1-3 artifacts are loaded into the shooter and at least one has settled in to the shooter itself.  all 3 flippers are retracted
    // Does not use sensors in any way, doesn't detect stalls, etc.
    // From the time the push is commanded to the ball hitting the flywheels is about 200-250ms
    public void shoot3SuperFast(boolean pushLeftFirst, boolean reset, boolean logShots, boolean auto, double distance) {
        superFastShooting.set(true);
        double maxAim = 1;
        long startTime = System.currentTimeMillis();
        teamUtil.log("shoot3SuperFast");
        if(!auto && distance < MID_DISTANCE_THRESHOLD) {
                lockShooter(distance);
                //maxAim = maxPitch(distance);
        }
        if(distance < Shooter.MID_DISTANCE_THRESHOLD){
            Shooter.SF_LEFT_PUSH_PAUSE = Shooter.SF_LEFT_PUSH_PAUSE_NEAR;
            Shooter.SF_RIGHT_PUSH_PAUSE = Shooter.SF_RIGHT_PUSH_PAUSE_NEAR;
        }else{
            Shooter.SF_LEFT_PUSH_PAUSE = Shooter.SF_LEFT_PUSH_PAUSE_FAR;
            Shooter.SF_RIGHT_PUSH_PAUSE = Shooter.SF_RIGHT_PUSH_PAUSE_FAR;
        }

        if (logShots) teamUtil.robot.logShot(leftFlywheel.getVelocity());
        pusher.push1NoWait();
        teamUtil.pause(SF_SHOT_PAUSE);

        if (pushLeftFirst) {
            leftPusher.setPosition(LEFT_PUSHER_PUSH);
            teamUtil.pause(SF_LEFT_PUSH_PAUSE);
            leftPusher.setPosition(LEFT_PUSHER_STOW);
            if (logShots) teamUtil.robot.logShot(leftFlywheel.getVelocity());
            if(!auto){aim(Math.min(aimer.getPosition()+SHOOT_3_AIM_CHANGE, maxAim));}
            pusher.push1NoWait();
            teamUtil.pause(SF_SHOT_PAUSE);

            rightPusher.setPosition(RIGHT_PUSHER_PUSH);
            teamUtil.pause(SF_RIGHT_PUSH_PAUSE);
            if (logShots) teamUtil.robot.logShot(leftFlywheel.getVelocity());
            if(!auto)aim(Math.min(aimer.getPosition()+SHOOT_3_AIM_CHANGE, maxAim));
            pusher.push1NoWait();
            teamUtil.pause(SF_LAST_SHOT_PAUSE);
            rightPusher.setPosition(RIGHT_PUSHER_STOW); // start moving this back so it doesn't trigger the loaded detector
            teamUtil.pause(SF_LAST_PADDLE_PAUSE);
        } else {
            rightPusher.setPosition(RIGHT_PUSHER_PUSH);
            teamUtil.pause(SF_RIGHT_PUSH_PAUSE);
            rightPusher.setPosition(RIGHT_PUSHER_STOW);
            if (logShots) teamUtil.robot.logShot(leftFlywheel.getVelocity());
            if(!auto)aim(aimer.getPosition()+SHOOT_3_AIM_CHANGE);
            pusher.push1NoWait();
            teamUtil.pause(SF_SHOT_PAUSE);

            leftPusher.setPosition(LEFT_PUSHER_PUSH);
            teamUtil.pause(SF_LEFT_PUSH_PAUSE);
            if (logShots) teamUtil.robot.logShot(leftFlywheel.getVelocity());
            if(!auto)aim(aimer.getPosition()+SHOOT_3_AIM_CHANGE);
            pusher.push1NoWait();
            teamUtil.pause(SF_LAST_SHOT_PAUSE);

            leftPusher.setPosition(LEFT_PUSHER_STOW);
            teamUtil.pause(SF_LAST_PADDLE_PAUSE); // start moving this back so it doesn't trigger the loaded detector
        }
        if (reset) pusher.reset(false);
        teamUtil.log("shoot3SuperFast Finished in " + (System.currentTimeMillis() - startTime));
        superFastShooting.set(false);
    }


    public void shootSuperFastNoWait (boolean pushLeftFirst, boolean reset, boolean logShots, boolean auto, double distance) {
        teamUtil.log("Launching Thread to shootSuperFastNoWait");
        if (superFastShooting.get()) {
            teamUtil.log("WARNING: shootSuperFastNoWait called while superFastShooting--Ignored");
            return;
        }
        superFastShooting.set(true);
        Thread thread = new Thread(new Runnable() {
            @Override
            public void run() {
                shoot3SuperFast(pushLeftFirst, reset, logShots, auto, distance);
            }
        });
        thread.start();

    }

    public void sidePushersStow() {
        leftPusher.setPosition(LEFT_PUSHER_STOW);
        rightPusher.setPosition(RIGHT_PUSHER_STOW);
    }
    public void sidePushersHold() {
        leftPusher.setPosition(LEFT_PUSHER_HOLD);
        rightPusher.setPosition(RIGHT_PUSHER_HOLD);
    }

    public static long LEFT_PUSH_PAUSE = 250;
    public static long RIGHT_PUSH_PAUSE = 250;

    public void pushLeft() {
        leftPusher.setPosition(LEFT_PUSHER_PUSH);
        teamUtil.pause(LEFT_PUSH_PAUSE);
        leftPusher.setPosition(LEFT_PUSHER_STOW);
    }
    public void pushRight() {
        rightPusher.setPosition(RIGHT_PUSHER_PUSH);
        teamUtil.pause(RIGHT_PUSH_PAUSE);
        rightPusher.setPosition(RIGHT_PUSHER_STOW);
    }


    // for aimer:
    // close: .000069913x + .218222
    // far: .0000328084x + .415
    // for v:
    // close: 0.349956x + 391+1/9
    // far: 0.131234x + 1300

    // distances:
    // 12.5ft for furthest
    // closest long shot: 10ft
    // closest close is 3ft 10in inches
    // furthest close shot is 91in
    public void adjustShooter(double distance){
        double velocityNeeded;
        double pitchNeeded;
        if (distance>MID_DISTANCE_THRESHOLD){
            velocityNeeded = 0.131234*distance + 1300;
            pitchNeeded = .0000328084*distance + .415;
        }else{
            velocityNeeded= 0.349956*distance + 391.1111111;
            pitchNeeded = .000069913*distance + .218222;
        }
        setShootSpeed(velocityNeeded);
        aim(pitchNeeded);

    }

    public static double A09_VELOCITY_A = 0.0000303382;
    public static double A09_VELOCITY_B = 0.13061;
    public static double A09_VELOCITY_C = 655.50668;

    /* OLD VALUES
    public static double A09_VELOCITY_A = 0.000239375;
    public static double A09_VELOCITY_B = -0.431755;
    public static double A09_VELOCITY_C = 984.33854;

     */


    public static double A09_SHORT_AIM_M = .000025;
    public static double A09_SHORT_AIM_B = .2775;
    public static double A09_LONG_AIM_M = .000118464;
    public static double A09_LONG_AIM_B = .18204;
//    public static double A09_DEEP_VELOCITY_M = 0.297372;
//    public static double A09_DEEP_VELOCITY_B = 472.75242;
    public static double A09_DEEP_VELOCITY_M = 0.15748;
    public static double A09_DEEP_VELOCITY_B = 1127.55906;

    public static double MANUAL_FLYWHEEL_ADJUST = 0;
    public static double LONG_MANUAL_FLYWHEEL_ADJUST = 0;

    public static double AMax = 0.0491219;
    public static double BMax = 0.0000140803;
    public static double CMax = -0.0124993;
    public static double DMax = 0.684305;

    public double maxPitch(double distance){
        return AMax * Math.log(BMax*distance+CMax) + DMax;
    }

    public double GetMaxFlywheelVelocity(double distance){
        return 0;
    }

//    public double getVelocityNeeded(double distance){
//        double velocityNeeded;
//        if (distance<MID_SHORT_DISTANCE_THRESHOLD){
//            velocityNeeded =A09_VELOCITY_A*Math.pow(distance,2) +A09_VELOCITY_B*distance + A09_VELOCITY_C;
//        }else if (distance<MID_DISTANCE_THRESHOLD){ // are the first two not the same?
//            velocityNeeded = A09_VELOCITY_A*Math.pow(distance,2) +A09_VELOCITY_B*distance + A09_VELOCITY_C;
//        }else{
//            velocityNeeded = Math.round((A09_DEEP_VELOCITY_M*distance + A09_DEEP_VELOCITY_B+LONG_MANUAL_FLYWHEEL_ADJUST) / 20.0) * 20.0;
//        }
//        return velocityNeeded + MANUAL_FLYWHEEL_ADJUST;
//    }

    /*
    public double getAimNeeded(double distance) {
        double pitchNeeded;
        if (distance < MID_SHORT_DISTANCE_THRESHOLD) {
            pitchNeeded = A09_SHORT_AIM_M * distance + A09_SHORT_AIM_B;
        } else if (distance < MID_DISTANCE_THRESHOLD) {
            pitchNeeded = A09_LONG_AIM_M * distance + A09_LONG_AIM_B;
        } else {
            pitchNeeded = 0.44;
        }
        return pitchNeeded;
    }

     */

/*
    public void adjustShooterV2(double distance){
        if(details)teamUtil.log("adjustShooterV2 to distance: " + distance);
        VELOCITY_COMMANDED = getVelocityNeeded(distance);
        setShootSpeed(VELOCITY_COMMANDED);
        aim(getAimNeeded(distance));
        if(details)teamUtil.log("adjustShooterV2 Finished");
    }

 */

//    public void adjustShooterV3(double distance){
//        if(details)teamUtil.log("adjustShooterV3 to distance: " + distance);
//
//        VELOCITY_COMMANDED = getVelocityNeeded(distance);
//        setShootSpeed(VELOCITY_COMMANDED);
//        aim(calculatePitch(distance, leftFlywheel.getVelocity()));
//        if(details)teamUtil.log("adjustShooterV3 Finished");
//    }

    public void adjustShooterV4(double distance){
        if(details)teamUtil.log("adjustShooterV4 to distance: " + distance);


        setShootSpeed(calculateVelocityV2(distance));
        aim(calculatePitchV2(distance));
        if(details)teamUtil.log("adjustShooterV4 Finished");
    }

    public static double minSpeedA = 0.0000490408;
    public static double minSpeedB = 0.0601474;
    public static double minSpeedC = 638.0056;

    public static double maxSpeedA = -0.000206498;
    public static double maxSpeedB = 1.31176;
    public static double maxSpeedC = -274.98039;

//    public double calculateMinSpeed(double distance){
//        return minSpeedA * distance * distance + minSpeedB * distance + minSpeedC;
//    }
//    public double calculateMaxSpeed(double distance){
//        return maxSpeedA * distance * distance + maxSpeedB * distance + maxSpeedC;
//    }
//    public double calculateMidSpeed(double distance){
//        return (calculateMinSpeed(distance) + calculateMaxSpeed(distance) ) / 2;
//    }

    public static double pitchA = 0;
    public static double pitchB = 0.0004007;
    public static double pitchC = 0;
    public static double pitchD = 9.91034e-8;
    public static double pitchE = 4.73733e-7;
    public static double pitchF = -6.27271e-7;

    /*
    public static double pitchA = 0.012;
    public static double pitchB = -0.0000044767;
    public static double pitchC = 0.000457262;
    public static double pitchD = -8.07697e-8;
    public static double pitchE = -3.01755e-7;
    public static double pitchF = 2.67729e-7;
    public static double longPitch = .48; // .44 with the old function

     */



    public static double LONG_MANUAL_PITCH_ADJUST = 0;

//    public double calculatePitch(double distance, double velocity) {
//        if(distance<MID_DISTANCE_THRESHOLD){
//            return pitchA + pitchB * distance + pitchC * velocity + pitchD * distance * distance + pitchE * velocity * velocity + pitchF * distance * velocity;
//        }else{
//            return longPitch + LONG_MANUAL_PITCH_ADJUST; // was the velocity really tuned with this in mind?
//        }
//    }

    public static double SHORT_DISTANCE_THRESHOLD = 1400;

    public static double pitchAShort = 2.27273e-7;
    public static double pitchBShort = -0.000393636;
    public static double pitchCShort = 0.466183;

    public static double pitchANew = -2.8869e-8;
    public static double pitchBNew = 0.000186393;
    public static double pitchCNew = 0.153392;
    public static double longPitch = .40; // .44 with the old function


    public double calculatePitchV2(double distance){
        if (distance<SHORT_DISTANCE_THRESHOLD) {
            return pitchAShort * distance * distance + pitchBShort * distance + pitchCShort;
        }if (distance<MID_DISTANCE_THRESHOLD){
            return pitchANew*distance*distance+pitchBNew*distance+pitchCNew;
        }else{
            return longPitch;
        }
    }

    public static double velocityANew = 0.000142857;
    public static double velocityBNew = -0.209524;
    public static double velocityCNew = 895.83333;

    public static double velocityAShort = 0.000454545;
    public static double velocityBShort = -0.787273;
    public static double velocityCShort = 1092.36364;

    public static double longVelocityM = 0.3;
    public static double longVelocityB = 540;
    public static double newLongVelocityM = 0.24698;
    public static double newLongVelocityB = 554.71749;

    public double calculateVelocityV2(double distance){
        if (distance<SHORT_DISTANCE_THRESHOLD){
            return velocityAShort*distance*distance+velocityBShort*distance+velocityCShort;
        }if (distance<MID_DISTANCE_THRESHOLD){
            return velocityANew*distance*distance+velocityBNew*distance+velocityCNew;
        }else{
            return newLongVelocityM*distance+newLongVelocityB;
        }
    }

    public void changeAim(double distance, double velocity){ // account for robot velocity?
        double angle = calculatePitchV2(distance);
        aim(angle);
    }

    public void lockShooter(double distance){
        double currentVelo = leftFlywheel.getVelocity();
        setShootSpeed(currentVelo); // Lock in the velocity to the current one (presumed to be in the OK range to shoot)
        double idealVelo = calculateVelocityV2(distance);
        double idealPitch = calculatePitchV2(distance);
        double targetPitch = idealPitch + ((currentVelo-idealVelo)/Robot.veloToPitchRatio);
        if(details){
            teamUtil.log("Ideal Pitch: " + idealPitch);
            teamUtil.log("New Pitch: " + targetPitch);
            teamUtil.log("Ideal Velo: " + idealVelo);
        }
        aim(targetPitch); // adjust the shooter pitch as needed to match the velocity
    }
    /*
    public boolean flywheelSpeedOK(double distance, double velocity){
        if(distance<MID_DISTANCE_THRESHOLD) {
            double minV = calculateMinSpeed(distance);
            double maxV = calculateMaxSpeed(distance);
            if (velocity > maxV || velocity < minV) { // not within thresholds
                return false;
            }
            return true;
        }else{
//            if(Math.abs(velocity-getVelocityNeeded(distance))>VELOCITY_COMMANDED_THRESHOLD){
//                return false;
//            }
//            return true;
            return Math.abs(velocity-getVelocityNeeded(distance)) < .0001;
        }
    }

     */

    /// ////////////////////////////////////////////////////////
    // NOT USED
    // New calibration for pusher: Stall pusher at an extreme pitch
/*
    public static float  PUSHER_CALIBRATE_POWER = .1f;
    public static int PUSHER_CALIBRATE_OFFSET = 500;
    public void calibratePusherV2() {
        pusher.CALIBRATED = false;
        long timeOutTime = System.currentTimeMillis() + 2000;
        teamUtil.log("Calibrating Pusher V2");
        aimer.setPosition(PUSHER_CALIBRATE_PITCH);
        teamUtil.pause(200); // wait for it to start moving

        pusher.servo.setPower(PUSHER_CALIBRATE_POWER);
        float lastPusherPosition = pusher.getPositionEncoder();
        teamUtil.pause(250);
        while (pusher.getPositionEncoder() != lastPusherPosition && teamUtil.keepGoing(timeOutTime)) {
            lastPusherPosition = pusher.getPositionEncoder();
            if (details) teamUtil.log("Calibrate Pusher: " + lastPusherPosition);
            teamUtil.pause(50);
        }
        pusher.servo.setPower(0);
        pusher.ENCODER_LOAD_POSITION_1 = pusher.getPositionEncoder() - PUSHER_CALIBRATE_OFFSET;

        aimer.setPosition(AIMER_CALIBRATE);
        teamUtil.pause(250);
        teamUtil.log("Calibrate PusherV2: ENCODER_LOAD_POSITION_1: "+ pusher.ENCODER_LOAD_POSITION_1);
        pusher.CALIBRATED = false;
    }

 */
}