package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.Disabled;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

@Disabled
@Autonomous(name="Auto Red", group="Linear Opmode")
public class AutoRed extends LinearOpMode {

    @Override
    public void runOpMode() {
        NyxTheRobot nyx = new NyxTheRobot(this);
        nyx.initialize();
        waitForStart();


        nyx.drive(-12, 0.5);
        nyx.setArm(-280);
        nyx.turn(100);
        nyx.drive(-24, 0.5);
        for (DcMotor m : nyx.AllMotors) {
            m.setPower(-0.025);
        }
        nyx.autoDucks(4, -0.35);
        for (DcMotor m : nyx.AllMotors) {
            m.setPower(0);
        }

        nyx.drive(2, 0.25);
        nyx.turn(-90);
        nyx.drive(-20, 0.5);

        nyx.setArm(0);

    }
}
