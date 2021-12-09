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

        while (opModeIsActive()){
            nyx.driverControl();
            nyx.ducks(gamepad1.dpad_left, gamepad1.dpad_right);
            nyx.setArm2(gamepad1.right_trigger);
//            nyx.lifty(gamepad2.right_trigger);

            if (gamepad1.a)
                nyx.intake(-gamepad1.left_trigger);
            else
                nyx.intake(gamepad1.left_trigger);
        }

    }
}
