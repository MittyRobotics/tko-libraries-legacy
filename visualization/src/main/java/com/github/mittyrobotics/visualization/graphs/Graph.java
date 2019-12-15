package com.github.mittyrobotics.visualization.graphs;

import com.github.mittyrobotics.visualization.util.XYLineShapeColorRenderer;
import com.github.mittyrobotics.visualization.util.XYSeriesCollectionWithRender;
import com.github.mittyrobotics.visualization.themes.GraphTheme;
import com.github.mittyrobotics.visualization.themes.TKOTheme;
import org.jfree.chart.ChartFactory;
import org.jfree.chart.ChartPanel;
import org.jfree.chart.JFreeChart;
import org.jfree.chart.axis.NumberAxis;
import org.jfree.chart.plot.PlotOrientation;
import org.jfree.chart.plot.XYPlot;
import org.jfree.data.xy.XYDataset;

import javax.swing.*;
import java.awt.*;
import java.text.DecimalFormat;

public class Graph extends JFrame {
	
	private String titleName;
	private String yAxisName;
	private String xAxisName;
	
	private XYSeriesCollectionWithRender[] datasets;
	
	private ChartPanel chartPanel;
	private JFreeChart chart;
	private XYPlot plot;
	
	public Graph(){
		this("Graph", "X","Y");
	}
	
	public Graph(String titleName, String yAxisName, String xAxisName) {
		this.titleName = titleName;
		this.yAxisName = yAxisName;
		this.xAxisName = xAxisName;
		
		initUI();
		setGraphTheme(new TKOTheme());
		setVisible(true);
	}
	
	private void initUI() {
		JFreeChart chart = createChart();

		ChartPanel chartPanel = new ChartPanel(chart);
		chartPanel.setBorder(BorderFactory.createEmptyBorder(15, 15, 15, 15));
		add(chartPanel);
		
		pack();
		setTitle(titleName);
		setLocationRelativeTo(null);
		setDefaultCloseOperation(JFrame.EXIT_ON_CLOSE);
		
		this.chartPanel = chartPanel;
	}
	
	private JFreeChart createChart() {
		
		JFreeChart chart = ChartFactory.createXYLineChart(
				titleName,
				xAxisName,
				yAxisName,
				null,
				PlotOrientation.VERTICAL,
				true,
				true,
				false
		);

		XYPlot plot = chart.getXYPlot();
		
		plot.setBackgroundPaint(Color.white);
		
		plot.setRangeGridlinesVisible(true);
		plot.setRangeGridlinePaint(Color.BLACK);
		
		plot.setDomainGridlinesVisible(true);
		plot.setDomainGridlinePaint(Color.BLACK);
		
		( (NumberAxis)plot.getDomainAxis()).setNumberFormatOverride(new DecimalFormat());
		( (NumberAxis)plot.getRangeAxis()).setNumberFormatOverride(new DecimalFormat());


		
		this.chart = chart;
		this.plot = plot;
		
		return chart;
	}
	
	public void setGraphTheme(GraphTheme theme){
		chartPanel.setBackground(theme.getFrameBackgroundColor());
		plot.setBackgroundPaint(theme.getGraphBackgroundColor());
		plot.setDomainGridlinePaint(theme.getGridColor());
		plot.setRangeGridlinePaint(theme.getGridColor());
		chart.setBackgroundPaint(theme.getFrameBackgroundColor());
		chart.getTitle().setPaint(theme.getTitleColor());
		chart.getLegend().setItemPaint(theme.getLabelColor());
		chart.getLegend().setBackgroundPaint(theme.getFrameBackgroundColor());
		plot.getDomainAxis().setLabelPaint(theme.getLabelColor());
		plot.getRangeAxis().setLabelPaint(theme.getLabelColor());
		plot.getDomainAxis().setTickLabelPaint(theme.getTickLabelColor());
		plot.getRangeAxis().setTickLabelPaint(theme.getTickLabelColor());
	}
	
	public void scaleGraphToFit(){
		double lowerBound = 9999;
		double upperBound = -9999;
		double leftBound = 9999;
		double rightBound = -9999;
		for(int i = 0; i < plot.getDatasetCount(); i++){
			for(int a = 0; a < plot.getDataset(i).getSeriesCount(); a++){
				for(int j = 0; j < plot.getDataset(i).getItemCount(a); j++){
					if(plot.getDataset(i).getYValue(a,j) < lowerBound){
						lowerBound = plot.getDataset(i).getYValue(a,j);
					}
					if(plot.getDataset(i).getYValue(a,j) > upperBound){
						upperBound = plot.getDataset(i).getYValue(a,j);
					}
					if(plot.getDataset(i).getXValue(a,j) < leftBound){
						leftBound = plot.getDataset(i).getXValue(a,j);
					}
					if(plot.getDataset(i).getXValue(a,j) > rightBound){
						rightBound = plot.getDataset(i).getXValue(a,j);
					}
				}
			}
		}
		
		double lowerRange=0;
		double upperRange=0;
		
		
		if(lowerBound < leftBound){
			lowerRange = lowerBound;
		}
		else{
			lowerRange = leftBound;
		}
		
		if(upperBound > rightBound){
			upperRange = upperBound;
		}
		else{
			upperRange = rightBound;
		}
		
		
		
		NumberAxis domain = (NumberAxis) plot.getDomainAxis();
		domain.setRange(lowerRange-20, upperRange+20);
		domain.setVerticalTickLabels(true);
		NumberAxis range = (NumberAxis) plot.getRangeAxis();
		range.setRange(lowerRange-20, upperRange+20);
		
	}
	
	public void resizeGraph(double lowerBound, double upperBound, double leftBound, double rightBound){
		NumberAxis domain = (NumberAxis) plot.getDomainAxis();
		domain.setRange(leftBound, rightBound);
		domain.setVerticalTickLabels(true);
		NumberAxis range = (NumberAxis) plot.getRangeAxis();
		range.setRange(lowerBound, upperBound);
	}
	
	public void clearGraph(){
		for(int i = 0; i < plot.getDatasetCount(); i++){
			plot.setDataset(0, null);
		}
	}
	
	public void updateGraph(){
		clearGraph();
		if(datasets != null){
			for(int i = 0; i < datasets.length; i++){
				plot.setDataset(i,datasets[i]);
				plot.setRenderer(i, new XYLineShapeColorRenderer(datasets[i].isShowPoints(), datasets[i].isShowLines(), datasets[i].getColor()));
			}
		}

		
	}
	
	public void setDatasets(XYSeriesCollectionWithRender[] datasets) {
		this.datasets = datasets;
		updateGraph();
	}

	public void addDataset(XYSeriesCollectionWithRender dataset){
		plot.setDataset(plot.getDatasetCount(),dataset);
		plot.setRenderer(plot.getDatasetCount()-1, new XYLineShapeColorRenderer(dataset.isShowPoints(), dataset.isShowLines(), dataset.getColor()));
	}
	
	public XYDataset[] getDatasets() {
		return datasets;
	}
	
	
	public ChartPanel getChartPanel() {
		return chartPanel;
	}
	
	public JFreeChart getChart() {
		return chart;
	}
	
	public XYPlot getPlot() {
		return plot;
	}
	

}
