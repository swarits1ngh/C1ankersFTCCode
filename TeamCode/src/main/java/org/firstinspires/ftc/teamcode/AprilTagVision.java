package org.firstinspires.ftc.teamcode;

import org.firstinspires.ftc.vision.apriltag.AprilTagDetection;
import org.firstinspires.ftc.vision.apriltag.AprilTagProcessor;

import java.util.List;

public class AprilTagVision {

    private final AprilTagProcessor aprilTag;

    public int lastSeenTagId = -1;
    public String lastSeenGoal = "NONE";

    public TagInfo.ObeliskPattern firstSeenPattern =
            TagInfo.ObeliskPattern.UNKNOWN;
    public TagInfo.ObeliskPattern currentPattern =
            TagInfo.ObeliskPattern.UNKNOWN;

    public AprilTagVision(AprilTagProcessor processor) {
        this.aprilTag = processor;
    }

    public void update() {
        List<AprilTagDetection> detections = aprilTag.getDetections();

        for (AprilTagDetection tag : detections) {

            int id = tag.id;
            lastSeenTagId = id;

            if (id == 20) lastSeenGoal = "BLUE";
            if (id == 24) lastSeenGoal = "RED";

            if (TagInfo.isObelisk(id)) {
                TagInfo.ObeliskPattern pattern = TagInfo.getPattern(id);

                if (firstSeenPattern == TagInfo.ObeliskPattern.UNKNOWN) {
                    firstSeenPattern = pattern;
                }
                currentPattern = pattern;
            }
        }
    }
}
