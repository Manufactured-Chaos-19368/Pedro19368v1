package pedroPathing.constants;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.localization.Pose;

@Config
public class FieldConstants {
    public static final Pose titanStartPose = new Pose(0, 0, Math.toRadians(0));
    public static final Pose leftSamplePreIntakePose = new Pose(-12, 11.5, Math.toRadians(130));
    public static final Pose leftSampleIntakePose = new Pose(-13, 15, Math.toRadians(125));
    public static final Pose midSamplePreIntakePose = new Pose(-16.5, 7.5, Math.toRadians(90));
    public static final Pose midSampleIntakePose = new Pose(-16.5, 15, Math.toRadians(90));
    public static final Pose midSampleIntakePoseFar = new Pose(-16.5, 15, Math.toRadians(90));
    public static final Pose rightSamplePreIntakePose = new Pose(-8, 7.5, Math.toRadians(90));
    public static final Pose rightSampleIntakePose = new Pose(-8, 15, Math.toRadians(90));
    public static final Pose rightSampleIntakePoseFar = new Pose(-8, 18, Math.toRadians(90));
    public static final Pose scorePose = new Pose(-16, 2.5, Math.toRadians(45));
    public static final Pose scorePose2 = new Pose(-18, 2.5, Math.toRadians(45));
    public static final Pose parkControlPose = new Pose(-16, 20, Math.toRadians(75));
    public static final Pose parkPose = new Pose(3, 50, Math.toRadians(0));

}

