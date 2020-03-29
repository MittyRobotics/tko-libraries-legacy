package com.github.mittyrobotics.datatypes.interfaces;

import java.util.concurrent.atomic.AtomicBoolean;

public interface InversionInterface {
    AtomicBoolean isInverted = new AtomicBoolean(false);
    default void setInverted(boolean inversion) {
        isInverted.set(inversion);
    }

    default boolean getInverted() {
        return isInverted.get();
    }
}