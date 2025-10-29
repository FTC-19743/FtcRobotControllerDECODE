package org.firstinspires.ftc.teamcode.assemblies;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.bosch.BNO055IMU;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;
//import org.firstinspires.ftc.teamcode.libs.Blinkin;
import org.firstinspires.ftc.teamcode.libs.teamUtil;

import java.util.concurrent.atomic.AtomicBoolean;

@Config
public class Robot {
    public BNO055IMU imu;
    HardwareMap hardwareMap;
    Telemetry telemetry;
    public BasicDrive drive;
    public Shooter output;

    public Intake intake;
    //public Blinkin blinkin;

    public static boolean details = false;


    public AtomicBoolean stopAutoOperations = new AtomicBoolean(false);

    public AtomicBoolean autoUnloadNoWaitDone = new AtomicBoolean(false);

    public AtomicBoolean doingUniversalDriveOp = new AtomicBoolean(true);

    public Robot() {
        telemetry = teamUtil.theOpMode.telemetry;
        hardwareMap = teamUtil.theOpMode.hardwareMap;
        drive = new BasicDrive();
        intake = new Intake();
        output = new Shooter();

        teamUtil.robot = this;

    }

    public void initialize() {
        drive.initalize();
        intake.initialize();
    }
    /*
    public void initCV (boolean enableLiveView) {
        intake.initCV(enableLiveView);
    }

     */

    public void outputTelemetry() {
        drive.driveMotorTelemetry();
        intake.intakeTelemetry();
        //output.outputTelemetry();
    }

    public void calibrate() {
        drive.calibrate();
        intake.calibrate();
        //output.calibrate();
    }

    public void resetRobot(){

    }

    /////////////////////////////////////////////////////////////////////////////////////////////////////////////
    /////////////////////////////////////////////////////////////////////////////////////////////////////////////
}

