package pedroPathing.constants;

import com.acmerobotics.dashboard.config.Config;
import com.pedropathing.localization.Pose;

@Config
public class FieldConstants {

    public static final Pose titanStartPose = new Pose(0, 0, Math.toRadians(0));

    //MOVE TO THIRD SAMPLE
    public static final Pose leftSamplePreIntakePose = new Pose(-12, 10.5, Math.toRadians(130));

    //MOVE INTO THIRD SAMPLE
    public static final Pose leftSampleIntakePose = new Pose(-13, 15, Math.toRadians(125));

    //MOVE TO SECOND SAMPLE
    public static final Pose midSamplePreIntakePose = new Pose(-17, 8.5, Math.toRadians(90));

    //MOVE INTO SECOND SAMPLE
    public static final Pose midSampleIntakePose = new Pose(-17, 18, Math.toRadians(90));

    //MOVE TO FIRST SAMPLE
    public static final Pose rightSamplePreIntakePose = new Pose(-8, 8.5, Math.toRadians(90));

    //MOVE INTO FIRST SAMPLE
    public static final Pose rightSampleIntakePose = new Pose(-8, 18, Math.toRadians(90));

    //SCORE SAMPLES
    public static final Pose scorePose = new Pose(-16, 2.5, Math.toRadians(45));

    //PARK
    public static final Pose parkControlPose = new Pose(-16, 20, Math.toRadians(75));
    public static final Pose parkPose = new Pose(-8, 36, Math.toRadians(0));

    //TELEOP
    public static final Pose scoreControlPose = new Pose(-16, 20, Math.toRadians(45));
    public static final Pose intakePose = new Pose(-5, 30, Math.toRadians(0));

}

