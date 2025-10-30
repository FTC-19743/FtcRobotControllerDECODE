package org.firstinspires.ftc.teamcode.libs;

import com.qualcomm.hardware.rev.RevBlinkinLedDriver;
import com.qualcomm.robotcore.hardware.HardwareMap;

import org.firstinspires.ftc.robotcore.external.Telemetry;

public class Blinkin {
    public RevBlinkinLedDriver blinkinLedDriver;
    public RevBlinkinLedDriver.BlinkinPattern current;
    private Telemetry telemetry;
    private HardwareMap hardwareMap;

    public boolean isFlashing = false;

    ///////////////////////////////////////////////////////////////////////////////
    //these are the slightly useful constants for certain patterns
    public final double larsonScanner = -0.01;

    public enum Signals{
        OFF,
        RED,
        FLASHING_RED,
        DARK_GREEN,
        YELLOW,
        GOLD,
        OCEANPALETTE,
        INIT_RED,
        INIT_BLUE,
        COLORWAVESFORESTPALETTE,
        SIGNAL_3,
        HEARTBEAT_WHITE,
        NORMAL_WHITE,


        VIOLET,
        VIOLETHEARTBEAT,
        MIDDLE,
        TOP,

        SPARKLY,

        RED_AUTO,
        LARSONSCANNERGRAY,

        DROPANDGOTOLOAD,

        RED_PATH_1,
        RED_PATH_2,
        RED_PATH_3,
        BLUE_PATH_1,
        BLUE_PATH_2,
        BLUE_PATH_3,
        JUDGING_BLINKIN,
        JUDGING_LEFT,
        JUDGING_RIGHT,

        GOTOSCORE_RED,

        GOTOSCORE_BLUE,
        SINELON_RED,
        SINELON_BLUE

    }

    public Blinkin(HardwareMap map, Telemetry aTelemetry ){
        hardwareMap = map;
        telemetry = aTelemetry;
    }
    public void init () {
        blinkinLedDriver = hardwareMap.get(RevBlinkinLedDriver.class, "blinkin");
        setSignal(Signals.OFF);
    }

    private void flashForMsecs(RevBlinkinLedDriver.BlinkinPattern pattern, long msecs) {
        long stopTime = System.currentTimeMillis() + msecs;
        blinkinLedDriver.setPattern(pattern);
        while (System.currentTimeMillis() < stopTime);
        blinkinLedDriver.setPattern(current);
        isFlashing = false;
    }

    public void flash (final RevBlinkinLedDriver.BlinkinPattern pattern, final long msecss) {
        if(isFlashing){
            return;
        }else {
            isFlashing = true;
            Thread thread = new Thread(new Runnable() {
                @Override
                public void run() {
                    flashForMsecs(pattern, msecss);
                }
            });
            thread.start();
        }
    }

    public void setSignal(Signals signal){
        //teamUtil.log("Blinkin: " + signal);

        switch(signal) {
            case OFF :
                current = RevBlinkinLedDriver.BlinkinPattern.BLACK;
                blinkinLedDriver.setPattern(current);
                break;

            case GOTOSCORE_BLUE:
                current = RevBlinkinLedDriver.BlinkinPattern.RAINBOW_OCEAN_PALETTE;
                blinkinLedDriver.setPattern(current);
                break;

            case FLASHING_RED:
                current = RevBlinkinLedDriver.BlinkinPattern.STROBE_RED;
                blinkinLedDriver.setPattern(current);
                break;

            case GOTOSCORE_RED:
                current = RevBlinkinLedDriver.BlinkinPattern.RAINBOW_LAVA_PALETTE;
                blinkinLedDriver.setPattern(current);
                break;


            case GOLD :
                current = RevBlinkinLedDriver.BlinkinPattern.GOLD;
                blinkinLedDriver.setPattern(current);
                break;
            case DROPANDGOTOLOAD:
                current = RevBlinkinLedDriver.BlinkinPattern.SINELON_RAINBOW_PALETTE;
                blinkinLedDriver.setPattern(current);
                break;
            case DARK_GREEN :
                current = RevBlinkinLedDriver.BlinkinPattern.DARK_GREEN;

                blinkinLedDriver.setPattern(current);
                break;
            case RED:
                current = RevBlinkinLedDriver.BlinkinPattern.RED;
                blinkinLedDriver.setPattern(current);
                break;

            case SINELON_BLUE:
                current = RevBlinkinLedDriver.BlinkinPattern.SINELON_OCEAN_PALETTE;
                blinkinLedDriver.setPattern(current);
                break;
            case SINELON_RED:
                current = RevBlinkinLedDriver.BlinkinPattern.SINELON_LAVA_PALETTE;
                blinkinLedDriver.setPattern(current);
                break;
            case OCEANPALETTE:
                current = RevBlinkinLedDriver.BlinkinPattern.BEATS_PER_MINUTE_OCEAN_PALETTE;
                blinkinLedDriver.setPattern(current);
                break;
            case COLORWAVESFORESTPALETTE :
                current = RevBlinkinLedDriver.BlinkinPattern.COLOR_WAVES_FOREST_PALETTE;
                blinkinLedDriver.setPattern(current);
                break;
            case INIT_RED:
                current = RevBlinkinLedDriver.BlinkinPattern.LARSON_SCANNER_RED;
                blinkinLedDriver.setPattern(current);
                break;
            case INIT_BLUE :
                current = RevBlinkinLedDriver.BlinkinPattern.SINELON_OCEAN_PALETTE;
                blinkinLedDriver.setPattern(current);
                break;
            case SIGNAL_3:
                current = RevBlinkinLedDriver.BlinkinPattern.RED;
                blinkinLedDriver.setPattern(current);
                break;

            case VIOLET:
                current = RevBlinkinLedDriver.BlinkinPattern.VIOLET;
                blinkinLedDriver.setPattern(current);
                break;


            case HEARTBEAT_WHITE:
                current = RevBlinkinLedDriver.BlinkinPattern.HEARTBEAT_WHITE;
                blinkinLedDriver.setPattern(current);
                break;

            case NORMAL_WHITE:
                current = RevBlinkinLedDriver.BlinkinPattern.WHITE;
                blinkinLedDriver.setPattern(current);
                break;
            case TOP:
                current = RevBlinkinLedDriver.BlinkinPattern.STROBE_GOLD;
                blinkinLedDriver.setPattern(current);
                break;

            case SPARKLY:
                current = RevBlinkinLedDriver.BlinkinPattern.RAINBOW_WITH_GLITTER;
                blinkinLedDriver.setPattern(current);
                break;

            case RED_AUTO:
                current = RevBlinkinLedDriver.BlinkinPattern.COLOR_WAVES_FOREST_PALETTE;
                blinkinLedDriver.setPattern(current);
                break;

            case LARSONSCANNERGRAY:
                current = RevBlinkinLedDriver.BlinkinPattern.LARSON_SCANNER_GRAY;
                blinkinLedDriver.setPattern(current);
                break;

            case RED_PATH_1:
                current = RevBlinkinLedDriver.BlinkinPattern.RED;
                blinkinLedDriver.setPattern(current);
                break;

            case RED_PATH_2:
                current = RevBlinkinLedDriver.BlinkinPattern.HEARTBEAT_RED;
                blinkinLedDriver.setPattern(current);
                break;

            case RED_PATH_3:
                current = RevBlinkinLedDriver.BlinkinPattern.BREATH_RED;
                blinkinLedDriver.setPattern(current);
                break;

            case BLUE_PATH_1:
                current = RevBlinkinLedDriver.BlinkinPattern.BLUE;
                blinkinLedDriver.setPattern(current);
                break;
            case BLUE_PATH_2:
                current = RevBlinkinLedDriver.BlinkinPattern.HEARTBEAT_BLUE;
                blinkinLedDriver.setPattern(current);
                break;
            case BLUE_PATH_3:
                current = RevBlinkinLedDriver.BlinkinPattern.BREATH_BLUE;
                blinkinLedDriver.setPattern(current);
                break;
            case JUDGING_BLINKIN:
                current = RevBlinkinLedDriver.BlinkinPattern.RAINBOW_RAINBOW_PALETTE;
                blinkinLedDriver.setPattern(current);
                break;
            case JUDGING_LEFT:
                current = RevBlinkinLedDriver.BlinkinPattern.RAINBOW_OCEAN_PALETTE;
                blinkinLedDriver.setPattern(current);
                break;
            case JUDGING_RIGHT:
                current = RevBlinkinLedDriver.BlinkinPattern.RAINBOW_LAVA_PALETTE;
                blinkinLedDriver.setPattern(current);
                break;
        }
    }
}
/*
1 1005 -0.99 Fixed Palette Pattern Rainbow, Rainbow Palette Pattern Density Speed Brightness
2 1015 -0.97 Fixed Palette Pattern Rainbow, Party Palette Pattern Density Speed Brightness
3 1025 -0.95 Fixed Palette Pattern Rainbow, Ocean Palette Pattern Density Speed Brightness
4 1035 -0.93 Fixed Palette Pattern Rainbow, Lave Palette Pattern Density Speed Brightness
5 1045 -0.91 Fixed Palette Pattern Rainbow, Forest Palette Pattern Density Speed Brightness
6 1055 -0.89 Fixed Palette Pattern Rainbow with Glitter Pattern Density Speed Brightness
7 1065 -0.87 Fixed Palette Pattern Confetti Pattern Density Speed Brightness
8 1075 -0.85 Fixed Palette Pattern Shot, Red - - Brightness
9 1085 -0.83 Fixed Palette Pattern Shot, Blue - - Brightness
10 1095 -0.81 Fixed Palette Pattern Shot, White - - Brightness
11 1105 -0.79 Fixed Palette Pattern Sinelon, Rainbow Palette Pattern Density Speed Brightness
12 1115 -0.77 Fixed Palette Pattern Sinelon, Party Palette Pattern Density Speed Brightness
13 1125 -0.75 Fixed Palette Pattern Sinelon, Ocean Palette Pattern Density Speed Brightness
14 1135 -0.73 Fixed Palette Pattern Sinelon, Lava Palette Pattern Density Speed Brightness
15 1145 -0.71 Fixed Palette Pattern Sinelon, Forest Palette Pattern Density Speed Brightness
16 1155 -0.69 Fixed Palette Pattern Beats per Minute, Rainbow Palette Pattern Density Speed Brightness
17 1165 -0.67 Fixed Palette Pattern Beats per Minute, Party Palette Pattern Density Speed Brightness
18 1175 -0.65 Fixed Palette Pattern Beats per Minute, Ocean Palette Pattern Density Speed Brightness
19 1185 -0.63 Fixed Palette Pattern Beats per Minute, Lava Palette Pattern Density Speed Brightness
20 1195 -0.61 Fixed Palette Pattern Beats per Minute, Forest Palette Pattern Density Speed Brightness
21 1205 -0.59 Fixed Palette Pattern Fire, Medium - - Brightness
22 1215 -0.57 Fixed Palette Pattern Fire, Large - - Brightness
23 1225 -0.55 Fixed Palette Pattern Twinkles, Rainbow Palette - - Brightness
24 1235 -0.53 Fixed Palette Pattern Twinkles, Party Palette - - Brightness
25 1245 -0.51 Fixed Palette Pattern Twinkles, Ocean Palette - - Brightness
26 1255 -0.49 Fixed Palette Pattern Twinkles, Lava Palette - - Brightness
27 1265 -0.47 Fixed Palette Pattern Twinkles, Forest Palette - - Brightness
28 1275 -0.45 Fixed Palette Pattern Color Waves, Rainbow Palette - - Brightness
29 1285 -0.43 Fixed Palette Pattern Color Waves, Party Palette - - Brightness
30 1295 -0.41 Fixed Palette Pattern Color Waves, Ocean Palette - - Brightness
31 1305 -0.39 Fixed Palette Pattern Color Waves, Lava Palette - - Brightness
32 1315 -0.37 Fixed Palette Pattern Color Waves, Forest Palette - - Brightness
33 1325 -0.35 Fixed Palette Pattern Larson Scanner, Red Pattern Width Speed Brightness
34 1335 -0.33 Fixed Palette Pattern Larson Scanner, Gray Pattern Width Speed Brightness
35 1345 -0.31 Fixed Palette Pattern Light Chase, Red Dimming Speed Brightness
36 1355 -0.29 Fixed Palette Pattern Light Chase, Blue Dimming Speed Brightness
37 1365 -0.27 Fixed Palette Pattern Light Chase, Gray Dimming Speed Brightness
38 1375 -0.25 Fixed Palette Pattern Heartbeat, Red - - Brightness
39 1385 -0.23 Fixed Palette Pattern Heartbeat, Blue - - Brightness
40 1395 -0.21 Fixed Palette Pattern Heartbeat, White - - Brightness
41 1405 -0.19 Fixed Palette Pattern Heartbeat, Gray - - Brightness
42 1415 -0.17 Fixed Palette Pattern Breath, Red - - Brightness
43 1425 -0.15 Fixed Palette Pattern Breath, Blue - - Brightness
44 1435 -0.13 Fixed Palette Pattern Breath, Gray - - Brightness
45 1445 -0.11 Fixed Palette Pattern Strobe, Red - - Brightness
46 1455 -0.09 Fixed Palette Pattern Strobe, Blue - - Brightness
47 1465 -0.07 Fixed Palette Pattern Strobe, Gold - - Brightness
48 1475 -0.05 Fixed Palette Pattern Strobe, White - - Brightness
49 1485 -0.03 Color 1 Pattern End to End Blend to Black - - Brightness
50 1495 -0.01 Color 1 Pattern Larson Scanner Pattern Width Speed Brightness
51 1505 0.01 Color 1 Pattern Light Chase Dimming Speed Brightness
52 1515 0.03 Color 1 Pattern Heartbeat Slow - - Brightness
53 1525 0.05 Color 1 Pattern Heartbeat Medium - - Brightness
54 1535 0.07 Color 1 Pattern Heartbeat Fast - - Brightness
55 1545 0.09 Color 1 Pattern Breath Slow - - Brightness
56 1555 0.11 Color 1 Pattern Breath Fast - - Brightness
57 1565 0.13 Color 1 Pattern Shot - - Brightness
58 1575 0.15 Color 1 Pattern Strobe - - Brightness
59 1585 0.17 Color 2 Pattern End to End Blend to Black - - Brightness
60 1595 0.19 Color 2 Pattern Larson Scanner Pattern Width Speed Brightness
61 1605 0.21 Color 2 Pattern Light Chase Dimming Speed Brightness
62 1615 0.23 Color 2 Pattern Heartbeat Slow - - Brightness
63 1625 0.25 Color 2 Pattern Heartbeat Medium - - Brightness
64 1635 0.27 Color 2 Pattern Heartbeat Fast - - Brightness
65 1645 0.29 Color 2 Pattern Breath Slow - - Brightness
66 1655 0.31 Color 2 Pattern Breath Fast - - Brightness
67 1665 0.33 Color 2 Pattern Shot - - Brightness
68 1675 0.35 Color 2 Pattern Strobe - - Brightness
69 1685 0.37 Color 1 and 2 Pattern Sparkle, Color 1 on Color 2 - - Brightness
70 1695 0.39 Color 1 and 2 Pattern Sparkle, Color 2 on Color 1 - - Brightness
71 1705 0.41 Color 1 and 2 Pattern Color Gradient, Color 1 and 2 - - Brightness
72 1715 0.43 Color 1 and 2 Pattern Beats per Minute, Color 1 and 2 Pattern Density Speed Brightness
73 1725 0.45 Color 1 and 2 Pattern End to End Blend, Color 1 to 2 - - Brightness
74 1735 0.47 Color 1 and 2 Pattern End to End Blend - - Brightness
75 1745 0.49 Color 1 and 2 Pattern Color 1 and Color 2 no blending
76 1755 0.51 Color 1 and 2 Pattern Twinkles, Color 1 and 2 - - Brightness
77 1765 0.53 Color 1 and 2 Pattern Color Waves, Color 1 and 2 - - Brightness
78 1775 0.55 Color 1 and 2 Pattern Sinelon, Color 1 and 2 Pattern Density Speed Brightness
79 1785 0.57 Solid Colors Hot Pink - - Brightness
80 1795 0.59 Solid Colors Dark red - - Brightness
81 1805 0.61 Solid Colors Red - - Brightness
82 1815 0.63 Solid Colors Red Orange - - Brightness
83 1825 0.65 Solid Colors Orange - - Brightness
84 1835 0.67 Solid Colors Gold - - Brightness
85 1845 0.69 Solid Colors Yellow - - Brightness
86 1855 0.71 Solid Colors Lawn Green - - Brightness
87 1865 0.73 Solid Colors Lime - - Brightness
88 1875 0.75 Solid Colors Dark Green - - Brightness
89 1885 0.77 Solid Colors Green - - Brightness
90 1895 0.79 Solid Colors Blue Green - - Brightness
91 1905 0.81 Solid Colors Aqua - - Brightness
92 1915 0.83 Solid Colors Sky Blue - - Brightness
93 1925 0.85 Solid Colors Dark Blue - - Brightness
94 1935 0.87 Solid Colors Blue - - Brightness
95 1945 0.89 Solid Colors Blue Violet - - Brightness
96 1955 0.91 Solid Colors Violet - - Brightness
97 1965 0.93 Solid Colors White - - Brightness
98 1975 0.95 Solid Colors Gray - - Brightness
99 1985 0.97 Solid Colors Dark Gray

 */