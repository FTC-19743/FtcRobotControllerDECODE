package org.firstinspires.ftc.teamcode.assemblies;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
import com.qualcomm.robotcore.hardware.DigitalChannel;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.libs.teamUtil;

@Config
public class Shooter {
    HardwareMap hardwareMap;
    Telemetry telemetry;
    public DcMotorEx leftFlywheel;
    public DcMotorEx rightFlywheel;
    public AxonPusher pusher;
    private Servo aimer;
    private DigitalChannel loadedSensor;
    private Servo rgb1;

    public boolean details;
    public static float AIMER_CALIBRATE = .4f;
    public static float AIMER_MIN = .3f;
    public static float AIMER_MAX = .55f;

    public static double SHOOTER_FAR_VELOCITY = 1300;
    public static float PUSHER_VELOCITY = .5f;

    //Old flywheel values pidf

    public static double shooterP = 50;
    public static double shooterI = 1;
    public static double shooterD = 0.8;
    public static double shooterF = 0;



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

    public static double MID_DISTANCE_THRESHOLD = 2667;
    public static double MID_SHORT_DISTANCE_THRESHOLD = 1400;

    public static double VELOCITY_COMMANDED;
    public static double VELOCITY_COMMANDED_THRESHOLD = 50;
    public static double IDLE_FLYWHEEL_VELOCITY = 800;

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
        pusher = new AxonPusher();
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

    public void initialize() {
        teamUtil.log("Initializing Shooter");
        leftFlywheel = hardwareMap.get(DcMotorEx.class,"leftflywheel");
        rightFlywheel = hardwareMap.get(DcMotorEx.class,"rightflywheel");
        leftFlywheel.setDirection(DcMotorSimple.Direction.REVERSE);
        flywheelStartup();
        pusher.initialize();
        aimer = hardwareMap.get(Servo.class,"aimer");
        loadedSensor = hardwareMap.get(DigitalChannel.class, "loaded");
        loadedSensor.setMode(DigitalChannel.Mode.INPUT);

        teamUtil.log("Shooter Initialized");
    }
    public void calibrate(){
        aimer.setPosition(AIMER_CALIBRATE);
        teamUtil.pause (500); // wait for right pitch before moving pusher
        pusher.setPower(0);
        pusher.calibrate();
        pushOne();
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
        pusher.setPower(-1);
        teamUtil.pause(pause);
        pusher.setPower(0);
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

    public static double A09_VELOCITY_A = 0.000239375;
    public static double A09_VELOCITY_B = -0.431755;
    public static double A09_VELOCITY_C = 984.33854;
    public static double A09_SHORT_AIM_M = .000025;
    public static double A09_SHORT_AIM_B = .2775;
    public static double A09_LONG_AIM_M = .000118464;
    public static double A09_LONG_AIM_B = .18204;
    public static double A09_DEEP_VELOCITY_M = 0.297372;
    public static double A09_DEEP_VELOCITY_B = 472.75242;

    public static double MANUAL_FLYWHEEL_ADJUST = 0;

    public double getVelocityNeeded(double distance){
        double velocityNeeded;
        if (distance<MID_SHORT_DISTANCE_THRESHOLD){
            velocityNeeded =A09_VELOCITY_A*Math.pow(distance,2) +A09_VELOCITY_B*distance + A09_VELOCITY_C;
        }else if (distance<MID_DISTANCE_THRESHOLD){ // are the first two not the same?
            velocityNeeded = A09_VELOCITY_A*Math.pow(distance,2) +A09_VELOCITY_B*distance + A09_VELOCITY_C;
        }else{
            velocityNeeded = A09_DEEP_VELOCITY_M*distance + A09_DEEP_VELOCITY_B; // was this tuned? it shouldn't be linear
        }
        return velocityNeeded + MANUAL_FLYWHEEL_ADJUST;
    }

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

    public void adjustShooterV3(double distance){
        if(details)teamUtil.log("adjustShooterV3 to distance: " + distance);

        VELOCITY_COMMANDED = getVelocityNeeded(distance);
        setShootSpeed(VELOCITY_COMMANDED);
        aim(calculatePitch(distance, leftFlywheel.getVelocity()));
        if(details)teamUtil.log("adjustShooterV3 Finished");
    }

    public static double minSpeedA = 0.0000490408;
    public static double minSpeedB = 0.0601474;
    public static double minSpeedC = 638.0056;

    public static double maxSpeedA = -0.000206498;
    public static double maxSpeedB = 1.31176;
    public static double maxSpeedC = -274.98039;

    public double calculateMinSpeed(double distance){
        return minSpeedA * distance * distance + minSpeedB * distance + minSpeedC;
    }
    public double calculateMaxSpeed(double distance){
        return maxSpeedA * distance * distance + maxSpeedB * distance + maxSpeedC;
    }
    public double calculateMidSpeed(double distance){
        return (calculateMinSpeed(distance) + calculateMaxSpeed(distance) ) / 2;
    }

    public static double pitchA = 0.012;
    public static double pitchB = -0.0000044767;
    public static double pitchC = 0.000457262;
    public static double pitchD = -8.07697e-8;
    public static double pitchE = -3.01755e-7;
    public static double pitchF = 2.67729e-7;
    public static double longPitch = .44;

    public static double LONG_MANUAL_PITCH_ADJUST = 0;

    public double calculatePitch(double distance, double velocity) {
        if(distance<MID_DISTANCE_THRESHOLD){
            return pitchA + pitchB * distance + pitchC * velocity + pitchD * distance * distance + pitchE * velocity * velocity + pitchF * distance * velocity;
        }else{
            return longPitch + LONG_MANUAL_PITCH_ADJUST; // was the velocity really tuned with this in mind?
        }
    }

    public void changeAim(double distance, double velocity){ // account for robot velocity?
        double angle = calculatePitch(distance, velocity);
        aim(angle);
    }

    public boolean flywheelSpeedOK(double distance, double velocity){
        if(distance<MID_DISTANCE_THRESHOLD) {
            double minV = calculateMinSpeed(distance);
            double maxV = calculateMaxSpeed(distance);
            if (velocity > maxV || velocity < minV) { // not within thresholds
                return false;
            }
            return true;
        }else{
            if(Math.abs(velocity-getVelocityNeeded(distance))>VELOCITY_COMMANDED_THRESHOLD){
                return false;
            }
            return true;
        }
    }
}