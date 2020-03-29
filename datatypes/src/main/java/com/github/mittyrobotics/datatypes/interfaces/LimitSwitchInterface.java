package com.github.mittyrobotics.datatypes.interfaces;

public interface LimitSwitchInterface {

    void configRoborioForwardLimitSwitch(int id, boolean inversion);

    void configRoborioReverseLimitSwitch(int id, boolean inversion);

    default void configRoborioForwardLimitSwitch(int id){
        configRoborioForwardLimitSwitch(id, false);
    }

    default void configRoborioReverseLimitSwitch(int id){
        configRoborioReverseLimitSwitch(id, false);
    }

    boolean getRoborioForwardLimitSwitch();

    boolean getRoborioReverseLimitSwitch();
}
