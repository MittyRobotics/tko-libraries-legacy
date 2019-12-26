/*
 * MIT License
 *
 * Copyright (c) 2019 Mitty Robotics (Team 1351)
 *
 * Permission is hereby granted, free of charge, to any person obtaining a copy
 * of this software and associated documentation files (the "Software"), to deal
 * in the Software without restriction, including without limitation the rights
 * to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
 * copies of the Software, and to permit persons to whom the Software is
 * furnished to do so, subject to the following conditions:
 *
 * The above copyright notice and this permission notice shall be included in all
 * copies or substantial portions of the Software.
 *
 * THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
 * IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
 * FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
 * AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
 * LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
 * OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
 * SOFTWARE.
 */

package com.github.mittyrobotics.simulation.util;

import javax.swing.*;
import java.awt.event.KeyAdapter;
import java.awt.event.KeyEvent;

public class SimOI extends JFrame implements Runnable {

    private static SimOI instance = new SimOI();


    private boolean upKey;
    private boolean downKey;
    private boolean leftKey;
    private boolean rightKey;

    private SimOI() {
        super("OI");
        setSize(200, 200);
        setVisible(true);
        addKeyListener(new KeyAdapter() {
            @Override
            public void keyTyped(KeyEvent e) {

            }

            public void keyReleased(KeyEvent e) {
                if (e.getKeyCode() == KeyEvent.VK_UP) upKey = false;
                if (e.getKeyCode() == KeyEvent.VK_DOWN) downKey = false;
                if (e.getKeyCode() == KeyEvent.VK_LEFT) leftKey = false;
                if (e.getKeyCode() == KeyEvent.VK_RIGHT) rightKey = false;
            }

            @Override
            public void keyPressed(KeyEvent e) {
                if (e.getKeyCode() == KeyEvent.VK_UP) upKey = true;
                if (e.getKeyCode() == KeyEvent.VK_DOWN) downKey = true;
                if (e.getKeyCode() == KeyEvent.VK_LEFT) leftKey = true;
                if (e.getKeyCode() == KeyEvent.VK_RIGHT) rightKey = true;
            }
        });
        new Thread(this).start();
    }

    public static SimOI getInstance() {
        return instance;
    }

    @Override
    public void run() {
        while (true) {
            try {
                Thread.sleep(20);
            } catch (InterruptedException e) {
                e.printStackTrace();
            }
        }
    }

    public boolean isUpKey() {
        return upKey;
    }

    public boolean isDownKey() {
        return downKey;
    }

    public boolean isLeftKey() {
        return leftKey;
    }

    public boolean isRightKey() {
        return rightKey;
    }
}
