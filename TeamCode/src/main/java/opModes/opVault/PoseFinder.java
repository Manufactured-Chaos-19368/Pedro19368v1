package opModes.opVault;

import com.pedropathing.follower.Follower;
import com.pedropathing.util.Constants;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;

import pedroPathing.constants.FConstants;
import pedroPathing.constants.LConstants;
import runModes.Auto;
@Disabled
@Autonomous(name = "Pose Finder", group = "Examples")
public class PoseFinder extends OpMode {
    public Auto auto;
    private Timer pathTimer, actionTimer, opmodeTimer;

    private int pathState;
    private double fastSpeed = .8;
    private double slowSpeed = .5;

    @Override
    public void init() {
        Constants.setConstants(FConstants.class, LConstants.class);
        auto =  new Auto(hardwareMap, telemetry, new Follower(hardwareMap));
        pathTimer = new Timer();
        opmodeTimer = new Timer();

        //auto.follower = new Follower(hardwareMap);
        //auto.follower.setStartingPose(titanStartPose);
    }

    /** This method is called once at the start of the OpMode.
     * It runs all the setup actions, including building paths and starting the path system **/
    @Override
    public void start() {
        opmodeTimer.resetTimer();
        auto.start();
        setPathState(0);
    }

    /** This is the main loop of the OpMode, it will run repeatedly after clicking "Play". **/
    @Override
    public void loop() { // These loop the movements of the robot
        auto.update();
        autonomousPathUpdate();

        // Feedback to Driver Hub
        telemetry.addData("x", auto.follower.getPose().getX());
        telemetry.addData("y", auto.follower.getPose().getY());
        telemetry.addData("heading", Math.toRadians(auto.follower.getPose().getHeading()));
        telemetry.update();
    }

    /** We do not use this because everything should automatically disable **/
    @Override
    public void stop() {
        //this could be our kill switch!
    }

    public void autonomousPathUpdate() {

    }

    /** These change the states of the paths and actions
     * It will also reset the timers of the individual switches **/
    public void setPathState(int pState) {
        pathState = pState;
        pathTimer.resetTimer();
    }
}

