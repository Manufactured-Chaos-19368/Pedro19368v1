package opModes;

import com.pedropathing.follower.Follower;
import com.pedropathing.util.Constants;
import com.pedropathing.util.Timer;
import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import  com.qualcomm.robotcore.eventloop.opmode.OpMode;

import pedroPathing.constants.FConstants;
import pedroPathing.constants.LConstants;
import runModes.Auto;


@Autonomous(name = "Auto2", group = "Examples")
public class CompAuto extends OpMode {
    public Auto auto;
    private Timer pathTimer, actionTimer, opmodeTimer;

    private boolean blockSensed = false;

    private int pathState;
    private double fastSpeed = 1;
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

//        if (auto.intakeTouchState()) {
//            if(pathState == 3) {
//                auto.follower.breakFollowing();
//                setPathState(4);
//            } else if (pathState == 6) {
//                auto.follower.breakFollowing();
//                setPathState(7);
//            } else if (pathState == 9) {
//                auto.follower.breakFollowing();
//                setPathState(10);
//            }
//        }

        // Feedback to Driver Hub
        telemetry.addData("path state", pathState);
        telemetry.addData("x", auto.follower.getPose().getX());
        telemetry.addData("y", auto.follower.getPose().getY());
        telemetry.addData("heading", auto.follower.getPose().getHeading());
        telemetry.update();
    }//

    /** We do not use this because everything should automatically disable **/
    @Override
    public void stop() {
        //this could be our kill switch!
    }

    public void autonomousPathUpdate() {
        switch (pathState) {

            case 0:  // MOVE TO SCORE PRELOAD.  RAISE LIFT AND PREP BUCKET.
                auto.liftPIDF = true;
                blockSensed = false;
                auto.follower.setMaxPower(fastSpeed);
                auto.startPreScore();
                auto.follower.followPath(auto.scorePreload, true);
                setPathState(1);
                break;

            case 1:  // DUMP PRELOAD INTO BASKET
                if(!auto.follower.isBusy() && auto.actionNotBusy()) {
                    auto.startDump();
                    setPathState(2);}
                break;

            case 2: // MOVE TO FIRST SAMPLE.  LOWER LIFT, SET BUCKET, AND REACH INTAKE
                if(!auto.follower.isBusy() && auto.actionNotBusy()) {
                    auto.liftPIDF = true;
                    //auto.setIntakeTouchToggle(false);
                    auto.startPostScore();
                    auto.follower.followPath(auto.prePickup1,true);
                    setPathState(3);
                }
                break;

            case 3: // MOVE INTO THE FIRST SAMPLE AND GRAB IT
                if(!auto.follower.isBusy() && auto.actionNotBusy()) {
                    auto.follower.setMaxPower(slowSpeed);
                    auto.follower.followPath(auto.grabPickup1, true);
                }
                break;

            case 50:
                if(!auto.follower.isBusy() && auto.actionNotBusy()) {
                    auto.follower.setMaxPower(slowSpeed);
                    auto.follower.followPath(auto.farPickup1, true);
                    setPathState(4);
                }
                break;

            case 4: // MOVE TO THE BASKET TO PREPARE TO SCORE FIRST SAMPLE
                if(!auto.follower.isBusy() && auto.actionNotBusy()) {
                    auto.follower.setMaxPower(fastSpeed);
                    auto.startTransfer();
                    blockSensed = false;
                    auto.follower.followPath(auto.scorePickup1,true);
                    setPathState(90);
                }
                break;

            case 90: // DUMP FIRST SAMPLE BUCKET
                if(!auto.follower.isBusy() && auto.actionNotBusy()) {
                    auto.startDump();
                    setPathState(5);}
                break;

            case 5: // MOVE TO SECOND SAMPLE.  LOWER LIFT, SET BUCKET, AND REACH INTAKE
                if(!auto.follower.isBusy() && auto.actionNotBusy()) {
                    //auto.setIntakeTouchToggle(false);
                    auto.startPostScore();
                    auto.follower.followPath(auto.prePickup2,true);
                    setPathState(6);
                }
                break;

                ///********* Moves to sample, if touch reads true skip case 51 and go to case 7 *********\\\
            case 6: // MOVE INTO THE SECOND SAMPLE AND GRAB IT.
                if(!auto.follower.isBusy() && auto.actionNotBusy()) {
                    auto.follower.setMaxPower(slowSpeed);
                    auto.follower.followPath(auto.grabPickup2,true);
                    setPathState(51);
                }
                break;

            case 51:
                if(!auto.follower.isBusy() && auto.actionNotBusy()) {
                    auto.follower.setMaxPower(slowSpeed);
                    auto.follower.followPath(auto.farPickup2, true);
                    setPathState(4);
                }
                break;

            case 7: // MOVE TO DEPOSIT IT.
                if(!auto.follower.isBusy() && auto.actionNotBusy()) {
                    auto.follower.setMaxPower(fastSpeed);
                    auto.startTransfer();
                    auto.follower.followPath(auto.scorePickup2, true);
                    setPathState(91);
                }
                break;

            case 91: // DUMP BUCKET
                if(!auto.follower.isBusy() && auto.actionNotBusy()) {
                    auto.startDump();
                    setPathState(8);
                }
                break;

            case 8: //MOVE TO SAMPLE 3
                if(!auto.follower.isBusy() && auto.actionNotBusy()) {
                    auto.startPostScore();
                    auto.follower.followPath(auto.prePickup3,true);
                    setPathState(9);
                }
                break;

            case 9: //MOVE INTO SAMPLE 3
                if(!auto.follower.isBusy() && auto.actionNotBusy()) {
                    auto.follower.setMaxPower(slowSpeed);
                    auto.follower.followPath(auto.grabPickup3,true);
                    setPathState(10);

                }
                break;

            case 10: // MOVE TO SCORE POSITION
                if(!auto.follower.isBusy() && auto.actionNotBusy()) {
                    auto.startTransfer();
                    blockSensed = false;
                    auto.follower.setMaxPower(fastSpeed);
                    auto.follower.followPath(auto.scorePickup3,true);
                    setPathState(92);
                }
                break;

            case 92: // DUMP BUCKET
                if(!auto.follower.isBusy() && auto.actionNotBusy()) {
                    auto.startDump();
                    //toggle = false;
                    setPathState(11);
                }
                break;

            case 11:  //PARK THE BOT
                if(auto.actionNotBusy() && auto.actionNotBusy()) {
                    auto.startPark();
                    auto.follower.followPath(auto.park,true);
                    setPathState(12);
                }
                break;

            case 12: // CLOSE OUT AUTO
                if(!auto.follower.isBusy() && auto.actionNotBusy()) {
                    setPathState(-1);
                }
                break;
        }
    }

    /** These change the states of the paths and actions
     * It will also reset the timers of the individual switches **/
    public void setPathState(int pState) {
        pathState = pState;
        pathTimer.resetTimer();
    }
}

