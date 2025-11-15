package org.firstinspires.ftc.teamcode.assemblies;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
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

    public boolean details;
    public static float AIMER_CALIBRATE = .4f;
    public static float AIMER_MIN = .3f;
    public static float AIMER_MAX = .55f;

    public static double SHOOTER_FAR_VELOCITY = 1300;
    public static float PUSHER_VELOCITY = .5f;

    public static double shooterP = 50;
    public static double shooterI = 1;
    public static double shooterD = 0.8;
    public static double shooterF = 0;

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

    public void initialize() {
        teamUtil.log("Initializing Shooter");
        leftFlywheel = hardwareMap.get(DcMotorEx.class,"leftflywheel");
        rightFlywheel = hardwareMap.get(DcMotorEx.class,"rightflywheel");
        leftFlywheel.setDirection(DcMotorSimple.Direction.REVERSE);
        leftFlywheel.setVelocityPIDFCoefficients(shooterP, shooterI, shooterD, shooterF);
        rightFlywheel.setVelocityPIDFCoefficients(shooterP, shooterI, shooterD, shooterF);
        pusher.initialize();
        aimer = hardwareMap.get(Servo.class,"aimer");


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

    public void aim (double pos) {
        if (pos<AIMER_MIN) pos = AIMER_MIN;
        if (pos>AIMER_MAX) pos = AIMER_MAX;
        aimer.setPosition(pos);
    }
    public double currentAim () {
        return aimer.getPosition();
    }
    public void pushOne(){
        pusher.pushN(1, AxonPusher.RTP_MAX_VELOCITY, 1500);
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

    public void adjustShooterV2(double distance){
        double velocityNeeded;
        double pitchNeeded;
        if (distance<MID_SHORT_DISTANCE_THRESHOLD){
            velocityNeeded = 0.000239375*Math.pow(distance,2) -0.431755*distance + 984.33854;
            pitchNeeded = .000025*distance + 0.2775;
        }else if (distance<MID_DISTANCE_THRESHOLD){
            velocityNeeded = 0.000239375*Math.pow(distance,2) -0.431755*distance + 984.33854;
            pitchNeeded = .000118464*distance + .18204;
        }else{
            velocityNeeded = 0.297372*distance + 472.75242;
            pitchNeeded = 0.44;
        }
        VELOCITY_COMMANDED = velocityNeeded;
        setShootSpeed(velocityNeeded);
        aim(pitchNeeded);

    }

}