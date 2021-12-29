package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name="One Person Drive", group="Linear Opmode")
public class SingleDrive extends LinearOpMode {

    @Override
    public void runOpMode() {
        NyxTheRobot nyx = new NyxTheRobot(this);
        nyx.initialize();
        waitForStart();
        float armPos = 0;
        boolean latch = false;

        while (opModeIsActive()){
            if (gamepad1.x) {
                latch = true;
                armPos = gamepad1.right_trigger;
            }
            else {
                if (latch)
                    armPos = ((float) nyx.ARM2.getCurrentPosition()) / 1725;
                latch = false;
            }

            nyx.driverControl();
            nyx.ducks(gamepad1.dpad_left, gamepad1.dpad_right);
            nyx.setArm2(armPos);
//            nyx.lifty(gamepad2.right_trigger);

            if (gamepad1.a)
                nyx.intake(-gamepad1.left_trigger);
            else
                nyx.intake(gamepad1.left_trigger);
        }

    }
}
