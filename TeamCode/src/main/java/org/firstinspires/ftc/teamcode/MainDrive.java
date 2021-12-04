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
            nyx.setArm2(gamepad2.right_trigger);
//            nyx.lifty(gamepad2.right_trigger);

            nyx.intake(gamepad2.left_stick_y);
        }

    }
}
