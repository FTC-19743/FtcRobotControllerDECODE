package org.firstinspires.ftc.teamcode.libs;

import com.qualcomm.hardware.lynx.LynxModule;
import com.qualcomm.hardware.lynx.commands.standard.LynxGetModuleStatusCommand;
import com.qualcomm.hardware.lynx.commands.standard.LynxGetModuleStatusResponse;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.util.RobotLog;

import org.firstinspires.ftc.robotcore.external.Telemetry;
import org.firstinspires.ftc.robotcore.external.navigation.CurrentUnit;
import org.firstinspires.ftc.robotcore.external.navigation.TempUnit;
import org.firstinspires.ftc.robotcore.external.navigation.VoltageUnit;
import org.firstinspires.ftc.teamcode.assemblies.Robot;

import java.util.List;

public class teamUtil {

    public static LinearOpMode theOpMode;
    public static boolean inInitialization = true;
    public static Telemetry telemetry;
    public static LynxModule chModule;
    public static LynxModule ehModule;
    public static boolean justRanAuto = false;
    public static boolean justRanCalibrateRobot = false;


    public enum Alliance {RED, BLUE}
    public static Alliance alliance = Alliance.RED;
    public enum Side {GOAL, HUMAN}
    public static Side SIDE = Side.GOAL;

    public static Robot robot;


    public static boolean finishedGoToLoad = true;

    public static boolean LEFT = true; // Don't use this anymore!

    public static long startTime;



    public static void init (LinearOpMode opMode) {
        theOpMode = opMode;
        telemetry = theOpMode.telemetry;
        List<LynxModule> modules = theOpMode.hardwareMap.getAll(LynxModule.class);
        if (modules.isEmpty()) {
            log("No Lynx modules found in the hardware map!");
        } else {
            log("Lynx modules found:");
            for (LynxModule module : modules) {
                log("- " + module.getDeviceName() + " Parent: " + module.isParent());
                if (module.isParent()) {
                    chModule = module;
                } else {
                    ehModule = module;
                }
            }
        }
        inInitialization = true;


    }

    public static void logOverTemp(String hub, LynxModule module) {
        LynxGetModuleStatusCommand command = new LynxGetModuleStatusCommand(module);
        try
        {
            LynxGetModuleStatusResponse response = command.sendReceive();
            String msg = new String("------------------------" + hub+ " MOTOR BRIDGES OVERTEMP! : ");
            boolean overtemp = false;
            for (int i = -0;i<4;i++) {
                if (response.isMotorBridgeOverTemp(i)) {
                    overtemp = true;
                    msg = msg + " Port " + i;
                }
            }
            if (overtemp) {
                log (msg);
            }
        }
        catch (Exception e)
        {
            log("Exception Caught in LogSystemHealth while checking" + hub + " status: " + e);
        }

    }
    public static void logSystemHealth () {
        log("ControlHub Temp: " + chModule.getTemperature(TempUnit.CELSIUS) + "C  Current: " + chModule.getCurrent(CurrentUnit.AMPS) + " Input Voltage: " + chModule.getInputVoltage(VoltageUnit.VOLTS));
        logOverTemp("Control Hub", chModule);
        log("ExpansionHub Temp: " + ehModule.getTemperature(TempUnit.CELSIUS) + "C  Current: " + ehModule.getCurrent(CurrentUnit.AMPS)+ " Input Voltage: " + chModule.getInputVoltage(VoltageUnit.VOLTS));
        logOverTemp("Expansion Hub", ehModule);

        List<String> warnings = chModule.getGlobalWarnings();
        for (String warning : warnings) {
            log("CH Warning: "+ warning);
        }
        warnings = ehModule.getGlobalWarnings();
        for (String warning : warnings) {
            log("EH Warning: "+ warning);
        }
    }

    // Wait for the specified milliseconds
    public static void pause(long sleepTime) {
        long wakeupTime= System.currentTimeMillis()+sleepTime;
        while(System.currentTimeMillis()< wakeupTime && !theOpMode.isStopRequested()){
            theOpMode.idle();
        }
    }

    // log something so we can filter out the FTC robot log info
    public static void log(String logString) {
        RobotLog.d("19743LOG:T" + Thread.currentThread().getId()+" "+ Thread.currentThread().getStackTrace()[3].getMethodName() + ": " + logString);
    }

    // A helper method to see if a long running operation should continue.
    // Call this in your loops
    public static boolean keepGoing(long timeOutTime) {
        String s=Long.toString(System.currentTimeMillis());

        //log(s);
        return ((inInitialization || theOpMode.opModeIsActive()) && (System.currentTimeMillis() < timeOutTime));
    }
}
