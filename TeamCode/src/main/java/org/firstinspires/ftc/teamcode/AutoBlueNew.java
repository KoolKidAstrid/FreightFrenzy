package org.firstinspires.ftc.teamcode;

import com.qualcomm.robotcore.eventloop.opmode.Autonomous;
import com.qualcomm.robotcore.eventloop.opmode.LinearOpMode;
import com.qualcomm.robotcore.hardware.DcMotor;

@Autonomous(name="Auto Blue New", group="Linear Opmode")
public class AutoBlueNew extends LinearOpMode {

    @Override
    public void runOpMode() {
        NyxTheRobot nyx = new NyxTheRobot(this);
        nyx.initialize();
        waitForStart();

//        drive to wobble thing
        nyx.drive(-13, 0.5);

//        raise head to wobble thing top layer
        while (nyx.ARM2.getCurrentPosition() > 1110 || nyx.ARM2.getCurrentPosition() < 1090)
            nyx.setArm2(1100f/1725f);
        nyx.ARM2.setPower(0);

//        deposit block
        nyx.intake(-0.5);
        sleep(1500);
        nyx.intake(0);

//        lower head
        while (nyx.ARM2.getCurrentPosition() > 50 || nyx.ARM2.getCurrentPosition() < -50)
            nyx.setArm2(0f/1725f);
        nyx.ARM2.setPower(0);

//        drive backwards
        nyx.drive(9, 0.5);

//        turn to carousel
        nyx.newTurn(90);

//        drive to carousel
        nyx.drive(48, 0.5);

//        spin :)
        nyx.autoDucks(5, 0.25);


//        turn to square
        nyx.drive(-1, 0.25);
        nyx.newTurn(-120);

//        park
        nyx.drive(-16, 0.5);
        nyx.newTurn(30);
        nyx.drive(-2, 0.5);

    }
}
