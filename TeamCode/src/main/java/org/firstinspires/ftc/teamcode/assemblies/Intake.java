package org.firstinspires.ftc.teamcode.assemblies;

import static androidx.core.math.MathUtils.clamp;

import android.graphics.Color;
import android.util.Size;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.hardware.AnalogInput;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.hardware.camera.CameraCharacteristics;
import org.firstinspires.ftc.robotcore.external.hardware.camera.CameraName;
import org.firstinspires.ftc.robotcore.external.hardware.camera.WebcamName;

//import org.firstinspires.ftc.teamcode.libs.Blinkin;

import org.firstinspires.ftc.teamcode.libs.teamUtil;
import org.firstinspires.ftc.vision.VisionPortal;

import java.util.concurrent.atomic.AtomicBoolean;

@Config // Makes Static data members available in Dashboard
public class Intake {
    HardwareMap hardwareMap;
    Telemetry telemetry;
    static public boolean details = false;

    public Servo left_flipper;
    public Servo middle_flipper;
    public Servo right_flipper;

    public DcMotorEx elevator;
    public DcMotorEx intake;

    private ColorSensor leftTopColorSensor;
    private ColorSensor middleTopColorSensor;
    private ColorSensor rightTopColorSensor;

    public AtomicBoolean moving = new AtomicBoolean(false);
    public AtomicBoolean timedOut = new AtomicBoolean(false);

    public enum ARTIFACT {NONE, GREEN, PURPLE};
    public ARTIFACT leftLoad,rightLoad,middleLoad;


    //TODO FIND OUT
    static public float ELEVATOR_TIC_PER_MM = 0000;
    static public float MM_PER_ELEVATOR_TIC = 0000;
    static public double ELEVATOR_CALIBRATE_POWER = -0.1;

    static public int ELEVATOR_GROUND = 5;
    static public int ELEVATOR_PRE_GROUND = 50;
    static public int ELEVATOR_UP = 500;
    static public int ELEVATOR_UP_THRESHOLD = 50;
    static public int ELEVATOR_DOWN_THRESHOLD = 50;
    static public int ELEVATOR_VELOCITY = 1500;





    //static public int ELEVATOR_TOLERANCE_RETRACT = 5;

    static public double INTAKE_IN_POWER = 0.7;
    static public double INTAKE_OUT_POWER = -0.7;



    //TODO FIND POSITIONS
    static public float FLIPPER_PRE_TRANSFER = 0.94f;
    static public float FLIPPER_CEILING = 0.8f;
    static public float FLIPPER_TRANSFER = 0.65f;
    static public float FLIPPER_SHOOTER_TRANSFER = 0.05f;


    public Intake() {
        teamUtil.log("Constructing Intake");
        hardwareMap = teamUtil.theOpMode.hardwareMap;
        telemetry = teamUtil.theOpMode.telemetry;
    }

    public void initialize() {
        teamUtil.log("Initializing Intake");

        intake = hardwareMap.get(DcMotorEx.class, "intake");
        elevator = hardwareMap.get(DcMotorEx.class, "elevator");

        left_flipper = hardwareMap.get(Servo.class,"leftflipper");
        right_flipper = hardwareMap.get(Servo.class,"rightflipper");
        middle_flipper = hardwareMap.get(Servo.class,"middleflipper");

        leftTopColorSensor = hardwareMap.get(RevColorSensorV3.class, "upperleftcolorsensor");
        middleTopColorSensor = hardwareMap.get(RevColorSensorV3.class, "uppermiddlecolorsensor");
        rightTopColorSensor = hardwareMap.get(RevColorSensorV3.class, "upperrightcolorsensor");

        teamUtil.log("Intake Initialized");
    }

    // Calibrate flippers and elevator.
    public void calibrate() {
        teamUtil.log("Calibrating Intake");
        calibrateElevators();
    }

    public void calibrateElevators(){
        teamUtil.log("Calibrating elevators Only");
        elevator.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        elevator.setPower(ELEVATOR_CALIBRATE_POWER);
        int lastelevatorPosition = elevator.getCurrentPosition();
        teamUtil.pause(250);
        while (elevator.getCurrentPosition() != lastelevatorPosition) { // TODO Add timeout and keepgoing check
            lastelevatorPosition = elevator.getCurrentPosition();
            if (details) teamUtil.log("Calibrate Intake: elevator: " + elevator.getCurrentPosition());
            teamUtil.pause(50);
        }
        elevator.setPower(0);
        teamUtil.pause(500); // let it "relax" just a bit
        elevator.setMode(DcMotorEx.RunMode.STOP_AND_RESET_ENCODER);
        //elevator.setTargetPositionTolerance(ELEVATOR_TOLERANCE_RETRACT);// make that our zero position
        elevator.setTargetPosition(ELEVATOR_GROUND);
        elevator.setMode(DcMotorEx.RunMode.RUN_TO_POSITION);
        elevator.setVelocity(0);

        teamUtil.log("Calibrate Intake Final: elevator: "+elevator.getCurrentPosition());
    }


    public void intakeTelemetry() {
        telemetry.addLine("Intake elevator Position: " + elevator.getCurrentPosition());
    }

    public void intakeIn(){
        intake.setPower(INTAKE_IN_POWER);
    }
    public void intakeOut(){
        intake.setPower(INTAKE_OUT_POWER);
    }
    public void intakeStop(){
        intake.setPower(0);
    }

    //TODO FIX METHOD
    public void elevatorToFlippers(){
        elevator.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        elevator.setVelocity(ELEVATOR_VELOCITY);
        left_flipper.setPosition(FLIPPER_PRE_TRANSFER);
        right_flipper.setPosition(FLIPPER_PRE_TRANSFER);
        middle_flipper.setPosition(FLIPPER_PRE_TRANSFER);
        teamUtil.pause(250);
        elevator.setTargetPosition(ELEVATOR_UP);
        while(Math.abs(elevator.getCurrentPosition()-ELEVATOR_UP)>ELEVATOR_UP_THRESHOLD){ // TODO Add timeout and keepgoing check
        }
        left_flipper.setPosition(FLIPPER_TRANSFER);
        right_flipper.setPosition(FLIPPER_TRANSFER);
        middle_flipper.setPosition(FLIPPER_TRANSFER);

        elevator.setTargetPosition(ELEVATOR_PRE_GROUND);
        while(Math.abs(elevator.getCurrentPosition()-ELEVATOR_PRE_GROUND)>ELEVATOR_DOWN_THRESHOLD){ // TODO Add timeout and keepgoing check
        }
        elevator.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        elevator.setPower(ELEVATOR_CALIBRATE_POWER);
        int lastelevatorPosition = elevator.getCurrentPosition();
        teamUtil.pause(250);
        while (elevator.getCurrentPosition() != lastelevatorPosition) { // TODO Add timeout and keepgoing check
            lastelevatorPosition = elevator.getCurrentPosition();
            if (details) teamUtil.log("Calibrate Intake: elevator: " + elevator.getCurrentPosition());
            teamUtil.pause(50);
        }
        elevator.setPower(0);
        moving.set(false);
    }
    public void elevatorToFlippersNoWait(){
        moving.set(true);
        teamUtil.log("Launching Thread to elevatorToFlippersNoWait.  Intake: moving = true");
        Thread thread = new Thread(new Runnable() {
            @Override
            public void run() {
                elevatorToFlippers();
            }
        });
        thread.start();
    }

    public void shootOne(){

    }

    public ARTIFACT checkLoadedArtifact(ColorSensor sensor) {
        return ARTIFACT.NONE; // TODO put sensing logic in
    }
    public void checkLoadedArtifacts () {
        leftLoad = checkLoadedArtifact(leftTopColorSensor);
        middleLoad = checkLoadedArtifact(middleTopColorSensor);
        rightLoad = checkLoadedArtifact(rightTopColorSensor);
    }
}