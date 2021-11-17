package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;

@TeleOp(name="One Person Drive", group="Linear Opmode")
public class MainDrive extends LinearOpMode {

    @Override
    public void runOpMode() {
        NyxTheRobot nyx = new NyxTheRobot(this);
        nyx.initialize();
        waitForStart();

        while (opModeIsActive()){
            nyx.driverControl();
            nyx.ducks(gamepad2.left_bumper, gamepad2.right_bumper);
            nyx.setArm((int) (-280 - (gamepad2.right_trigger * -280)));
//            nyx.lifty(gamepad2.right_trigger);

            nyx.spinny(gamepad2.left_stick_y);
        }

    }
}
