package org.firstinspires.ftc.teamcode;

public class TagInfo {

    public enum ObeliskPattern {
        GREEN_PURPLE_PURPLE,
        PURPLE_GREEN_PURPLE,
        PURPLE_PURPLE_GREEN,
        UNKNOWN
    }

    public static boolean isObelisk(int id) {
        return id == 21 || id == 22 || id == 23;
    }

    public static String getLabel(int id) {
        switch (id) {
            case 20: return "BLUE GOAL";
            case 21: return "G-P-P OBELISK";
            case 22: return "P-G-P OBELISK";
            case 23: return "P-P-G OBELISK";
            case 24: return "RED GOAL";
            default: return "UNKNOWN";
        }
    }

    public static ObeliskPattern getPattern(int id) {
        switch (id) {
            case 21: return ObeliskPattern.GREEN_PURPLE_PURPLE;
            case 22: return ObeliskPattern.PURPLE_GREEN_PURPLE;
            case 23: return ObeliskPattern.PURPLE_PURPLE_GREEN;
            default: return ObeliskPattern.UNKNOWN;
        }
    }
}
