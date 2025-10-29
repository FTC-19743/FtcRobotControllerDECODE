package org.firstinspires.ftc.teamcode.assemblies;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.robotcore.hardware.CRServo;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.DcMotorSimple;
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
    public AxonPusher pusher = new AxonPusher();
    public Servo aimer;

    public boolean details;

    static public float AIMER_START = 00f;
    public static double PUSHER_INCREMENT = 0;
    public static double AIMER_CALIBRATE = .5;
    public static double SHOOTER_FAR_VELOCITY = 1300;
    public static float PUSHER_VELOCITY = .5f;


    public Shooter() {
        teamUtil.log("Constructing Shooter");
        hardwareMap = teamUtil.theOpMode.hardwareMap;
        telemetry = teamUtil.theOpMode.telemetry;
    }

    public void initalize() {
        teamUtil.log("Initializing Shooter");
        leftFlywheel = hardwareMap.get(DcMotorEx.class,"leftflywheel");
        rightFlywheel = hardwareMap.get(DcMotorEx.class,"rightflywheel");
        leftFlywheel.setDirection(DcMotorSimple.Direction.REVERSE);

        pusher.init(hardwareMap, "pusher");
        aimer = hardwareMap.get(Servo.class,"aimer");


        teamUtil.log("Shooter Initialized");
    }
    public void calibrate(){
        aimer.setPosition(AIMER_CALIBRATE);
        pusher.setPower(0);
    }
    public void outputTelemetry(){
        telemetry.addLine("Left velocity: "+leftFlywheel.getVelocity() +" Right velocity: "+rightFlywheel.getVelocity());
        telemetry.addLine("Pusher Position: "+leftFlywheel.getVelocity()+" Aimer position: "+aimer.getPosition());
        telemetry.update();
    }
    public void setShootSpeed(double velocity){
        leftFlywheel.setVelocity(velocity);
        rightFlywheel.setVelocity(velocity);
    }
    public void stopShootSpeed(double velocity){
        leftFlywheel.setVelocity(0);
        rightFlywheel.setVelocity(0);
    }

    public void pushOne(){
        pusher.runToEncoderPosition(pusher.getPositionEncoder()+PUSHER_INCREMENT,PUSHER_VELOCITY,1000);
    }
}