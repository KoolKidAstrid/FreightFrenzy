package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name="Two Person Drive", group="Linear Opmode")
public class MainDrive extends LinearOpMode {

    @Override
    public void runOpMode() {
        NyxTheRobot nyx = new NyxTheRobot(this);
        nyx.initialize();
        waitForStart();
        float armPos = 0;
        boolean latch = false;

        while (opModeIsActive()){
            if (gamepad2.x) {
                latch = true;
                armPos = gamepad2.right_trigger;
            }
            else {
                if (latch)
                    armPos = ((float) nyx.ARM2.getCurrentPosition()) / 1725;
                latch = false;
            }


            nyx.driverControl();
            nyx.ducks(gamepad2.dpad_left, gamepad2.dpad_right);
            nyx.setArm2(armPos);
//            nyx.lifty(gamepad2.right_trigger);

            nyx.intake(gamepad2.left_stick_y);
        }

    }
}
