module tko.libraries.visualization.main {
	requires java.datatransfer;
	requires java.desktop;
	requires jfreechart;
	requires tko.libraries.datatypes.main;
	
	exports com.github.mittyrobotics.visualization.graphs;
	exports com.github.mittyrobotics.visualization.util;
}