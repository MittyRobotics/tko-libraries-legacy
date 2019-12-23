module tko.libraries.path.generation.main {
	requires java.datatransfer;
	requires java.desktop;
	requires jfreechart;
	requires tko.libraries.datatypes.main;
	requires tko.libraries.visualization.main;
	
	exports com.github.mittyrobotics.path.generation.datatypes;
	exports com.github.mittyrobotics.path.generation.paths;
}