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

        pusher.initialize();
        aimer = hardwareMap.get(Servo.class,"aimer");


        teamUtil.log("Shooter Initialized");
    }
    public void calibrate(){
        aimer.setPosition(AIMER_CALIBRATE);
        pusher.setPower(0);
        pusher.calibrate();
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
}