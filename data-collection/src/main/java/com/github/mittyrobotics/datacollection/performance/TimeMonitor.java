package com.github.mittyrobotics.datacollection.performance;

public class TimeMonitor {
	private long absoluteStartTime;
	private long absoluteEndTime;
	private long timeNanos;
	private String name;
	
	public TimeMonitor(){
		this.name = "task";
	}
	
	public TimeMonitor(String name){
		this.name = name;
	}
	
	public void start(){
		this.absoluteStartTime = System.nanoTime();
	}
	
	public void end(){
		this.absoluteEndTime = System.nanoTime();
		this.timeNanos = absoluteEndTime-absoluteStartTime;
	}
	
	public double getNanos(){
		if(timeNanos == 0){
			System.out.println("Time has not been measured because it has either not started or stopped");
		}
		return timeNanos;
	}
	
	public double getMillis(){
		if(timeNanos == 0){
			System.out.println("Time has not been measured because it has either not started or stopped");
		}
		return timeNanos/1000000.0;
	}
	
	public void printNanos(){
		if(timeNanos == 0){
			System.out.println("Time has not been measured because it has either not started or stopped");
		}
		System.out.println("Ellapsed time in nanoseconds to perform task " + name + ": " + timeNanos + " nanoseconds.");
	}
	
	public void printMillis(){
		if(timeNanos == 0){
			System.out.println("Time has not been measured because it has either not started or stopped");
		}
		System.out.println("Ellapsed time in milliseconds to perform task " + name + ": " + timeNanos/1000000.0 + " milliseconds.");
	}
	
}
