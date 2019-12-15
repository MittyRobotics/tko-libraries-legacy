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
