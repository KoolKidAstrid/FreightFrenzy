package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.eventloop.opmode.TeleOp;
import com.qualcomm.robotcore.hardware.DcMotor;
import com.qualcomm.robotcore.util.ElapsedTime;
import com.qualcomm.robotcore.util.Range;

@TeleOp(name="One Person Drive", group="Linear Opmode")
public class MainDrive extends LinearOpMode {

    @Override
    public void runOpMode() {
        SwaggyTheRobot swaggy = new SwaggyTheRobot(this);
        swaggy.initialize();
        waitForStart();

        while (opModeIsActive()){
            swaggy.driverControl();
            swaggy.ducks(gamepad2.left_bumper, gamepad2.right_bumper);
        }

    }
}
