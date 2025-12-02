package org.firstinspires.ftc.teamcode.assemblies;

import static androidx.core.math.MathUtils.clamp;

import com.acmerobotics.dashboard.config.Config;
import com.qualcomm.hardware.rev.RevColorSensorV3;
import com.qualcomm.robotcore.hardware.ColorSensor;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.hardware.DcMotorEx;
import com.qualcomm.robotcore.hardware.HardwareMap;
import com.qualcomm.robotcore.hardware.Servo;

import org.firstinspires.ftc.robotcore.external.Telemetry;

//import org.firstinspires.ftc.teamcode.libs.Blinkin;

import org.firstinspires.ftc.teamcode.libs.Blinkin;
import org.firstinspires.ftc.teamcode.libs.teamUtil;

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

    private ColorSensor leftLowerColorSensor;
    private ColorSensor middleLowerColorSensor;
    private ColorSensor rightLowerColorSensor;

    public AtomicBoolean elevatorMoving = new AtomicBoolean(false);
    public AtomicBoolean failedOut = new AtomicBoolean(false);

    public AtomicBoolean detecting = new AtomicBoolean(false);
    public AtomicBoolean stopDetector = new AtomicBoolean(false);

    public enum ARTIFACT {NONE, GREEN, PURPLE};
    public ARTIFACT leftLoad,rightLoad,middleLoad, leftIntake, middleIntake, rightIntake;
    public int intakeNum = 0;

    //TODO FIND OUT
    static public double ELEVATOR_CALIBRATE_POWER = -0.1;
    static public int ELEVATOR_GROUND = 5;
    static public long ELEVATOR_PAUSE_1 = 500;
    static public long ELEVATOR_PAUSE_2 = 500;
    public static float ELEVATOR_UP_POWER = .5f;
    public static float ELEVATOR_DOWN_POWER = -.5f;
    public static int ELEVATOR_UP_TIMEOUT = 1500;
    public static int ELEVATOR_DOWN_TIMEOUT = 1500;
    public static int ELEVATOR_UP_VELOCITY_THRESHOLD = 400;
    public static int ELEVATOR_DOWN_VELOCITY_THRESHOLD = -600;

    public static int ELEVATOR_REVERSE_INTAKE_ENCODER = 140;
    public static int ELEVATOR_UP_ENCODER = 440;
    public static int ELEVATOR_DOWN_ENCODER = 130;
    public static int ELEVATOR_UNLOAD_ENCODER = 460;
    public static int ELEVATOR_HOLD_VELOCITY = 1500;
    public static long ELEVATOR_STARTUP_TIME = 250;

    static public double INTAKE_IN_POWER = 0.7;
    static public double INTAKE_OUT_POWER = -0.7;

    static public float FLIPPER_PRE_TRANSFER = 0.94f;
    static public float FLIPPER_CEILING = 0.8f;
    static public float FLIPPER_TRANSFER = 0.7f;
    static public float MIDDLE_FLIPPER_SHOOTER_TRANSFER = 0.05f;
    static public float EDGE_FLIPPER_SHOOTER_TRANSFER = 0.15f;


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

        leftLowerColorSensor = hardwareMap.get(ColorSensor.class, "lowerleftcolorsensor");
        middleLowerColorSensor = hardwareMap.get(ColorSensor.class, "lowermiddlecolorsensor");
        rightLowerColorSensor = hardwareMap.get(ColorSensor.class, "lowerrightcolorsensor");

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


    private String formatSensor (ColorSensor sensor) {
        return String.format ("(%d/%d/%d/%d)",sensor.alpha(), sensor.red(), sensor.green(), sensor.blue());
    }
    public void intakeTelemetry() {
        telemetry.addLine("Intake elevator Position: " + elevator.getCurrentPosition());
        telemetry.addData("UpperSensors(A/R/G/B): ", "L%s M%s R%s",formatSensor(leftTopColorSensor), formatSensor(middleTopColorSensor), formatSensor(rightTopColorSensor));
        telemetry.addData("LowerSensors(A/R/G/B): ", "L%s M%s R%s",formatSensor(leftLowerColorSensor), formatSensor(middleLowerColorSensor), formatSensor(rightLowerColorSensor));
        checkLoadedArtifacts();
        checkIntakeArtifacts();
        telemetry.addLine("Loaded: L:" + leftLoad.toString() + "  M:" + middleLoad.toString() + "  R:" + rightLoad.toString());
        telemetry.addLine("Intake: Num:" + intakeNum + " L:" + leftIntake.toString() + "  M:" + middleIntake.toString() + "  R:" + rightIntake.toString());
    }

    public void intakeIn(){
        intake.setPower(INTAKE_IN_POWER);
    }
    public void intakeStart(){
        intakeIn();
        startDetector(false);
        left_flipper.setPosition(FLIPPER_CEILING);
        right_flipper.setPosition(FLIPPER_CEILING);
        middle_flipper.setPosition(FLIPPER_CEILING);

    }
    public void intakeOut(){
        intake.setPower(INTAKE_OUT_POWER);
    }
    public void intakeStop(){
        intake.setPower(0);
    }

    public void flippersToTransfer() {
        left_flipper.setPosition(FLIPPER_TRANSFER);
        right_flipper.setPosition(FLIPPER_TRANSFER);
        middle_flipper.setPosition(FLIPPER_TRANSFER);
    }

    public boolean elevatorToGroundV2() {
        teamUtil.log("elevatorToGroundV2.");
        failedOut.set(false);
        elevatorMoving.set(true);
        if (elevator.getCurrentPosition() <= 5) {
            teamUtil.log("WARNING: elevatorToGroundV2 called while elevator already at bottom--Ignored");
            elevatorMoving.set(false);
            return true;
        }
        elevator.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        elevator.setPower(ELEVATOR_DOWN_POWER);
        long timeOutTime = System.currentTimeMillis()+ELEVATOR_DOWN_TIMEOUT;
        teamUtil.pause(250); // allow time for elevator to get moving
        while(teamUtil.keepGoing(timeOutTime) && elevator.getVelocity() < ELEVATOR_DOWN_VELOCITY_THRESHOLD && elevator.getCurrentPosition()>ELEVATOR_DOWN_ENCODER){
            if (details) {
                teamUtil.log("Elevator Pos: "+ elevator.getCurrentPosition() + " Vel: "+ elevator.getVelocity());
            }
        }
        if (elevator.getCurrentPosition() > ELEVATOR_DOWN_ENCODER) {
            // We ran into an issue so fail out
            teamUtil.log("WARNING: elevatorToGroundV2 Failed Out due to stall or timeout");
            elevator.setPower(0);
            elevatorMoving.set(false);
            failedOut.set(true);
            return false;
        }

        // Run the last bit at low power and stall against mechanical block
        elevator.setMode(DcMotorEx.RunMode.RUN_USING_ENCODER);
        elevator.setPower(ELEVATOR_CALIBRATE_POWER);
        int lastelevatorPosition = elevator.getCurrentPosition();
        teamUtil.pause(250);
        while (teamUtil.keepGoing(timeOutTime) && elevator.getCurrentPosition() != lastelevatorPosition) {
            lastelevatorPosition = elevator.getCurrentPosition();
            if (details) teamUtil.log("dropping Intake: elevator: " + elevator.getCurrentPosition());
            teamUtil.pause(50);
        }
        elevator.setPower(0);
        elevatorMoving.set(false);

        if (System.currentTimeMillis() > timeOutTime) {
            teamUtil.log("elevatorToGroundV2 TIMED OUT!!!!!!!!!!!!");
            failedOut.set(true);
            return false;
        } else {
            teamUtil.log("elevatorToGroundV2 Finished");
            return true;
        }
    }

    public boolean getReadyToIntake() {
        teamUtil.log("getReadyToIntake");

        // Move flippers into out of the way position to drop any artifacts we might be holding
        left_flipper.setPosition(FLIPPER_PRE_TRANSFER);
        right_flipper.setPosition(FLIPPER_PRE_TRANSFER);
        middle_flipper.setPosition(FLIPPER_PRE_TRANSFER);
        teamUtil.pause(500); // Allow time for drop

        // Move elevator to ground
        if( elevatorToGroundV2()) {
            // Move flippers into blocking position
            left_flipper.setPosition(FLIPPER_CEILING);
            right_flipper.setPosition(FLIPPER_CEILING);
            middle_flipper.setPosition(FLIPPER_CEILING);

            intakeIn();
            teamUtil.log("getReadyToIntake Finished");
            return true;
        } else {
            teamUtil.log("getReadyToIntake Finished");
            return false;
        }
    }

    public void getReadyToIntakeNoWait(){
        if (elevatorMoving.get()) {
            teamUtil.log("WARNING: getReadyToIntake called while moving--Ignored");
            return;
        }
        elevatorMoving.set(true);
        failedOut.set(false);
        teamUtil.log("Launching Thread to getReadyToIntake.");
        Thread thread = new Thread(new Runnable() {
            @Override
            public void run() {
                getReadyToIntake();
            }
        });
        thread.start();
    }

    public boolean elevatorToFlippersV2(){
        teamUtil.log("elevatorToFlippersV2NoWait");

        failedOut.set(false);
        elevatorMoving.set(true);
        if (elevator.getCurrentPosition() > ELEVATOR_DOWN_ENCODER) {
            teamUtil.log("WARNING: elevatorToFlippersV2 called while elevator not at bottom--Ignored");
            elevatorMoving.set(false);
            return true;
        }

        // TODO: Consider turning the intake wheel off at this point, it might make it slightly more difficult for the elevator to go up but avoid pulling in extra balls

        // Move flippers out of the way
        left_flipper.setPosition(FLIPPER_PRE_TRANSFER);
        right_flipper.setPosition(FLIPPER_PRE_TRANSFER);
        middle_flipper.setPosition(FLIPPER_PRE_TRANSFER);
        teamUtil.pause(ELEVATOR_PAUSE_1);

        // Move elevator up while checking for stall (jam) or timeout
        elevator.setMode(DcMotor.RunMode.RUN_USING_ENCODER);
        elevator.setPower(ELEVATOR_UP_POWER);
        long timeOutTime = System.currentTimeMillis()+ELEVATOR_UP_TIMEOUT;
        teamUtil.pause(ELEVATOR_STARTUP_TIME); // allow time for elevator to get moving
        while(teamUtil.keepGoing(timeOutTime) && elevator.getVelocity() > ELEVATOR_UP_VELOCITY_THRESHOLD && elevator.getCurrentPosition()<ELEVATOR_UP_ENCODER){
            if (details) {
                teamUtil.log("Elevator Pos: "+ elevator.getCurrentPosition() + " Vel: "+ elevator.getVelocity());
            }
            // If the balls are past the intake wheel, reverse the intake to avoid pulling in more
            if (elevator.getCurrentPosition() > ELEVATOR_REVERSE_INTAKE_ENCODER) {
                intakeOut();
            }
        }
        if (elevator.getCurrentPosition() < ELEVATOR_UP_ENCODER) {
            // We ran into an issue so fail out
            teamUtil.log("elevatorToFlippersV2 Failed Out due to time out or stall");

            elevator.setPower(0);
            stopDetector();
            elevatorMoving.set(false);
            failedOut.set(true);
            return false;
        }
        stopDetector();

        // Hold at unload position
        elevator.setTargetPosition(ELEVATOR_UNLOAD_ENCODER);
        elevator.setMode(DcMotor.RunMode.RUN_TO_POSITION);
        elevator.setVelocity(ELEVATOR_HOLD_VELOCITY);
        elevatorMoving.set(false);

        // Load balls into flippers
        left_flipper.setPosition(FLIPPER_TRANSFER);
        right_flipper.setPosition(FLIPPER_TRANSFER);
        middle_flipper.setPosition(FLIPPER_TRANSFER);
        teamUtil.pause(ELEVATOR_PAUSE_2);

        boolean success =  elevatorToGroundV2();
        teamUtil.log("elevatorToFlippersV2 Finished");
        return success;
    }

    public void elevatorToFlippersV2NoWait(){
        if (elevatorMoving.get()) {
            teamUtil.log("WARNING: elevatorToFlippersV2NoWait called while moving--Ignored");
            return;
        }
        elevatorMoving.set(true);
        failedOut.set(false);
        teamUtil.log("Launching Thread to elevatorToFlippersV2NoWait.");
        Thread thread = new Thread(new Runnable() {
            @Override
            public void run() {
                elevatorToFlippersV2();
            }
        });
        thread.start();
    }

    public static double FLIPPERS_UNLOAD = 0.5;
    public static int UNLOAD_PAUSE = 1000;
    public static int SHORT_UNLOAD_PAUSE = 1000;
    public Servo onTheBackburner;
    public void unloadToShooter(boolean ordered){ // TODO: check if the artifacts are loaded before sending them to the shooter
        unloadOrder()[0].setPosition(FLIPPERS_UNLOAD);
        if(unloadOrder()[0] == middle_flipper){teamUtil.pause(SHORT_UNLOAD_PAUSE);
        }else{teamUtil.pause(UNLOAD_PAUSE);}
        unloadOrder()[1].setPosition(FLIPPERS_UNLOAD);
        onTheBackburner = unloadOrder()[2];
    }

    public Servo[] unloadOrder(){
        //todo: implement
        Servo[] arr = {left_flipper, middle_flipper, right_flipper};
        return arr;
    }
    public static int UPPER_ALPHA_THRESHOLD = 25;
    public static double GREEN_THRESHOLD = 1.25;
    public static int LEFT_ALPHA_THRESHOLD = 52;

    public ARTIFACT checkLoadedArtifact(ColorSensor sensor) {
        if(sensor.alpha() < UPPER_ALPHA_THRESHOLD){return ARTIFACT.NONE;}
        if(sensor == leftTopColorSensor && sensor.alpha() < LEFT_ALPHA_THRESHOLD){return ARTIFACT.NONE;}
        if((float) sensor.green()/ (float) sensor.alpha() > GREEN_THRESHOLD){return ARTIFACT.GREEN;}
        return ARTIFACT.PURPLE;
    }

    public static int LOWER_ALPHA_THRESHOLD = 75;

    public int loadedBallNum(){
        checkLoadedArtifacts();
        int loadedBalls = 0;
        if(leftLoad != ARTIFACT.NONE){
            loadedBalls++;
        }
        if(middleLoad != ARTIFACT.NONE){
            loadedBalls++;
        }
        if(rightLoad != ARTIFACT.NONE){
            loadedBalls++;
        }
        return loadedBalls;
    }

    public int intakeBallNum(){
        checkIntakeArtifacts();
        int intakeBalls = 0;
        if(leftIntake != ARTIFACT.NONE){
            intakeBalls++;
        }
        if(middleIntake != ARTIFACT.NONE){
            intakeBalls++;
        }
        if(rightIntake != ARTIFACT.NONE){
            intakeBalls++;
        }
        return intakeBalls;
    }

    public ARTIFACT checkIntakeArtifact(ColorSensor sensor) {
        /*
        int color = colorSensor.argb();
        int alpha = (color >> 24) & 0xFF;
        int red   = (color >> 16) & 0xFF;
        int green = (color >> 8)  & 0xFF;
        int blue  =  color        & 0xFF;
         */

        if(sensor.alpha() < LOWER_ALPHA_THRESHOLD){return ARTIFACT.NONE;}
        if (sensor.red() > sensor.green() || sensor.blue() > sensor.green()) {return ARTIFACT.PURPLE;}
        return ARTIFACT.GREEN;
    }
    public void checkLoadedArtifacts () {
        leftLoad = checkLoadedArtifact(leftTopColorSensor);
        middleLoad = checkLoadedArtifact(middleTopColorSensor);
        rightLoad = checkLoadedArtifact(rightTopColorSensor);
    }

    public void checkIntakeArtifacts () {
        leftIntake = checkIntakeArtifact(leftLowerColorSensor);
        middleIntake = checkIntakeArtifact(middleLowerColorSensor);
        rightIntake = checkIntakeArtifact(rightLowerColorSensor);
        intakeNum = (leftIntake == ARTIFACT.NONE ? 0 : 1) + (middleIntake == ARTIFACT.NONE ? 0 : 1) + (rightIntake == ARTIFACT.NONE ? 0 : 1);
    }

    public void setBlinkinArtifact(ARTIFACT artifact){
        switch (artifact) {
            case NONE: teamUtil.robot.blinkin.setSignal(Blinkin.Signals.EMPTY); break;
            case PURPLE: teamUtil.robot.blinkin.setSignal(Blinkin.Signals.PURPLE); break;
            case GREEN: teamUtil.robot.blinkin.setSignal(Blinkin.Signals.GREEN); break;
        }
    }

    public static int FLASH_TIME = 100;
    public static int GAP_TIME = 100;
    public static int CYCLE_TIME = 500;
    public void detectIntakeArtifacts(boolean colors) {
        while (!stopDetector.get()) {
            checkIntakeArtifacts();
            if (colors) {
                setBlinkinArtifact(leftIntake);
                teamUtil.pause(FLASH_TIME);
                teamUtil.robot.blinkin.setSignal(Blinkin.Signals.OFF);
                teamUtil.pause(GAP_TIME);
                setBlinkinArtifact(middleIntake);
                teamUtil.pause(FLASH_TIME);
                teamUtil.robot.blinkin.setSignal(Blinkin.Signals.OFF);
                teamUtil.pause(GAP_TIME);
                setBlinkinArtifact(rightIntake);
                teamUtil.pause(FLASH_TIME);
                teamUtil.robot.blinkin.setSignal(Blinkin.Signals.OFF);
                teamUtil.pause(CYCLE_TIME);
            } else {
                switch (intakeNum) {
                    case 0 : teamUtil.robot.blinkin.setSignal(Blinkin.Signals.ZERO);break;
                    case 1 : teamUtil.robot.blinkin.setSignal(Blinkin.Signals.ONE);break;
                    case 2 : teamUtil.robot.blinkin.setSignal(Blinkin.Signals.TWO);break;
                    case 3 : teamUtil.robot.blinkin.setSignal(Blinkin.Signals.THREE);break;
                }
            }
        }
        detecting.set(false);
        stopDetector.set(false);
    }

    public void startDetector (boolean colors) {
        if (detecting.get()) {
            teamUtil.log ("WARNING----------startDetector called while already detecting. Ignored");
        } else {
            detecting.set(true);
            stopDetector.set(false);
            teamUtil.log("Launching Thread to startDetector.");
            Thread thread = new Thread(new Runnable() {
                @Override
                public void run() {
                    detectIntakeArtifacts(colors);
                }
            });
            thread.start();

        }
    }

    public void stopDetector () {
        if (!detecting.get()) {
            teamUtil.log ("WARNING----------stopDetector called while not detecting. Ignored");
        } else {
            stopDetector.set(true);
            teamUtil.log("Stopping Detector Thread");
            teamUtil.robot.blinkin.setSignal(Blinkin.Signals.OFF);
        }
    }

}