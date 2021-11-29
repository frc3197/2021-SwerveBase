// Copyright (c) FIRST and other WPILib contributors.
// Open Source Software; you can modify and/or share it under the terms of
// the WPILib BSD license file in the root directory of this project.

package frc.robot.util;

import edu.wpi.first.wpilibj.XboxController;
import edu.wpi.first.wpilibj.GenericHID.Hand;

/** Add your docs here. */
public class FilteredController {
    private XboxController controller;

    public FilteredController(XboxController controller) {
        this.controller = controller;
    }

    /**
     * Gets the filtered X input for the given stick.
     * 
     * @param hand
     * @param deadzone
     * @return double
     */
    public double getX(Hand hand, double deadzone) {
        return new InputFilter(controller.getX(hand)).getFiltered(deadzone);
    }

    /**
     * Gets the filtered Y input for the given stick.
     * 
     * @param hand
     * @param deadzone
     * @return double
     */
    public double getY(Hand hand, double deadzone) {
        return new InputFilter(controller.getY(hand)).getFiltered(deadzone);
    }

    /**
     * Gets the filtered trigger input for the given trigger.
     * 
     * @param hand
     * @param deadzone
     * @return double
     */
    public double getTrigger(Hand hand, double deadzone) {
        return new InputFilter(controller.getTriggerAxis(hand)).getFiltered(deadzone);
    }
    public boolean getPOVPressed(){
        return controller.getPOVCount() > 0;
    }

    public int getPOVButton() {
        int POVButton;
        if (controller.getPOVCount() >= 1) {
            switch (controller.getPOV()) {
                case 0:
                    POVButton = 8;
                    break;

                case 45:
                    POVButton = 9;
                    break;

                case 90:
                    POVButton = 6;
                    break;

                case 135:
                    POVButton = 3;
                    break;

                case 180:
                    POVButton = 2;
                    break;

                case 225:
                    POVButton = 1;
                    break;

                case 270:
                    POVButton = 4;
                    break;

                case 315:
                    POVButton = 7;
                    break;

                case 360:
                    POVButton = 8;
                    break;
                default:
                    POVButton = 0;
            }
            return POVButton;
        } else {
            return 0;
        }
    }
}
