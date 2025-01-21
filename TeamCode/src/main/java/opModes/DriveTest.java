package opModes;

import static pedroPathing.constants.FieldConstants.parkPose;

import com.pedropathing.follower.Follower;
import com.pedropathing.util.Constants;
import com.qualcomm.robotcore.eventloop.opmode.OpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

import runModes.StolenTele;
import pedroPathing.constants.FConstants;
import pedroPathing.constants.LConstants;
//@Disabled

@TeleOp(name="DriveTest", group="A")
public class DriveTest extends OpMode {

    private StolenTele stolenTele;

    @Override
    public void init() {
        Constants.setConstants(FConstants.class, LConstants.class);
        stolenTele = new StolenTele(hardwareMap, telemetry, new Follower(hardwareMap), parkPose, false, gamepad1, gamepad2);
        //teleop.init();
    }

    @Override
    public void start() {
        stolenTele.start();
    }

    @Override
    public void loop() {
        stolenTele.update();
    }

}

