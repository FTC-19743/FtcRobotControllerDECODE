package org.firstinspires.ftc.teamcode.assemblies;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.teamcode.libs.Blinkin;
import org.firstinspires.ftc.teamcode.libs.teamUtil;

import java.util.concurrent.atomic.AtomicBoolean;

@Config
public class Robot {
    public BNO055IMU imu;
    HardwareMap hardwareMap;
    Telemetry telemetry;
    public OctoQuadFWv3 oq;
    public BasicDrive drive;
    public Intake intake;
    public Shooter shooter;
    public Blinkin blinkin;
    public Servo foot;
    public static double FOOT_CALIBRATE_POS = .1;
    public static double FOOT_EXTENDED_POS = .8;

    public static boolean details = false;

    // Set teamUtil.theOpMode before calling
    public Robot() {
        telemetry = teamUtil.theOpMode.telemetry;
        hardwareMap = teamUtil.theOpMode.hardwareMap;
        drive = new BasicDrive();
        intake = new Intake();
        shooter = new Shooter();
        foot = hardwareMap.get(Servo.class,"pushup");
        blinkin = new Blinkin(hardwareMap,telemetry);
        teamUtil.robot = this;
    }

    public void initialize() {
        drive.initalize();
        intake.initialize();
        shooter.initialize();
    }

    public void outputTelemetry() {
        drive.driveMotorTelemetry();
        intake.intakeTelemetry();
        shooter.outputTelemetry();
        telemetry.update();
    }

    public void calibrate() {
        drive.calibrate();
        intake.calibrate();
        shooter.calibrate();
    }

    public void setFootPos(double pos){
        foot.setPosition(pos);
    }

    public void resetRobot(){

    }

    /////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /////////////////////////////////////////////////////////////////////////////////////////////////////////////
}

