package org.firstinspires.ftc.teamcode.opModes.Auto;

/**
 * Enum representing the alliance color.
 * Used to determine the color of the alliance for the robot.
 */
public enum AllianceColor {
    None(20),
    Blue(20),
    Red(24);

    final int tagID;
    AllianceColor(int tagID) {
        this.tagID = tagID;
    }

    public int getTagID() {
        return tagID;
    }
}