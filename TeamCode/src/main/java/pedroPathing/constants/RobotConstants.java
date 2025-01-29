package pedroPathing.constants;

public class RobotConstants {
    //****Linkage****\\
    static public double LLinkageIn = 0.245;
    static public double RLinkageIn = 0.03;
    static public double LLinkagePartial= 0.32; //3
    static public double RLinkagePartial= 0.101; //
    public static double LLinkageMax = 0.495;
    static public double RLinkageMax = 0.272;
    public static double LinkageManualIncrements = .003;
    public static double LinkageMax = LLinkageMax;
    public static double LinkageMin = LLinkageIn;

    //****Bucket****\\
    static public double BucketMid= 0.4;
    static public double BucketDeposit= 0.7;
    static public double BucketTransfer= 0;

    //****Lift****\\
    static public double liftSpeed = 1;
    static public int LiftZero = 0;
    static public int LiftRest= 20;
    static public int LiftTransfer= 300;//290
    static public int LiftHighBasket= 2300;
    static public int LiftLowBasket= 800;//26.5
    static public int liftAdjust = 0;

    //****Wrist****\\
    static public double WristIn= 0.018;
    static public double WristMid= 0.373;
    static public double WristOut= 0.555;

    //****Intake****\\
    static public double intakeSpinInPwr = 1;
    static public double intakeSpinOutPwr = -1;
    static public double intakeSpinStopPwr = 0;

    //****Drive****\\
    static public double DRIVE_SPEED_FULL = .90;
    static public final double DRIVE_SPEED_SLOW = .4;
    static public final double DRIVE_SPEED_MEDIUM = 0.60;
    public double driveSpeed = DRIVE_SPEED_FULL;

    //****Auto****\\
    static public double fastSpeed = 1;
    static public double slowSpeed = 1;
}
