package com.github.mittyrobotics.datatypes.interfaces;

import edu.wpi.first.wpilibj.PowerDistributionPanel;

public class TKOPDP extends PowerDistributionPanel {

    private static TKOPDP instance;

    private TKOPDP(){
        super();
    }

    public static TKOPDP getInstance(){
        if(instance == null){
            instance = new TKOPDP();
        }
        return instance;
    }
}