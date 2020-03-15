/*
 * MIT License
 *
 * Copyright (c) 2020 Mitty Robotics (Team 1351)
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

package com.github.mittyrobotics.visualization;

import com.github.mittyrobotics.visualization.themes.DefaultDarkTheme;
import com.github.mittyrobotics.visualization.themes.GraphTheme;
import org.jfree.chart.ChartFactory;
import org.jfree.chart.ChartPanel;
import org.jfree.chart.JFreeChart;
import org.jfree.chart.axis.NumberAxis;
import org.jfree.chart.plot.PlotOrientation;
import org.jfree.chart.plot.XYPlot;
import org.jfree.data.xy.XYSeries;
import org.jfree.data.xy.XYSeriesCollection;

import javax.swing.BorderFactory;
import javax.swing.JFrame;
import java.awt.Color;
import java.awt.EventQueue;
import java.text.DecimalFormat;

public class Graph extends JFrame {

    public Graph() {
        this("Graph", "y", "x");
    }

    public Graph(String titleName, String yAxisName, String xAxisName) {
        this.titleName = titleName;
        this.yAxisName = yAxisName;
        this.xAxisName = xAxisName;

        initUI();
        setGraphTheme(new DefaultDarkTheme());
        setVisible(true);
    }

    private String titleName;
    private String yAxisName;
    private String xAxisName;
    private JFreeChart chart;
    private XYPlot plot;
    private ChartPanel chartPanel;
    private XYSeriesCollection defaultDataset = new XYSeriesCollection();

    private void initUI() {
        JFreeChart chart = createChart();

        ChartPanel chartPanel = new ChartPanel(chart);
        chartPanel.setBorder(BorderFactory.createEmptyBorder(0, 0, 0, 30));
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

        ((NumberAxis) plot.getDomainAxis()).setNumberFormatOverride(new DecimalFormat());
        ((NumberAxis) plot.getRangeAxis()).setNumberFormatOverride(new DecimalFormat());

        this.chart = chart;
        this.plot = plot;

        return chart;
    }

    public void addSeriesToDefaultDataset(XYSeries series){
        defaultDataset.addSeries(series);
        plot.setDataset(defaultDataset);
    }

    private int addIndex = 1;
    public void addDataset(XYSeriesCollectionWithRenderer dataset){
        plot.setDataset(addIndex, dataset);
        getPlot().setRenderer(addIndex,
                new XYLineShapeColorRenderer(dataset.isShowPoints(), dataset.isShowLines(), dataset.getColor()));
        addIndex++;
    }

    public void clearGraph() {
        for (int i = 1; i < getPlot().getDatasetCount(); i++) {
            getPlot().setDataset(i, null);
        }
        addIndex = 3;
    }


    /**
     * Scales the graph uniformly to fit all points on it
     */
    public void scaleGraphToFit() {
        double lowerBound = 9999;
        double upperBound = -9999;
        double leftBound = 9999;
        double rightBound = -9999;
        for (int i = 0; i < plot.getDatasetCount(); i++) {
            for (int a = 0; a < plot.getDataset(i).getSeriesCount(); a++) {
                for (int j = 0; j < plot.getDataset(i).getItemCount(a); j++) {
                    if (plot.getDataset(i).getYValue(a, j) < lowerBound) {
                        lowerBound = plot.getDataset(i).getYValue(a, j);
                    }
                    if (plot.getDataset(i).getYValue(a, j) > upperBound) {
                        upperBound = plot.getDataset(i).getYValue(a, j);
                    }
                    if (plot.getDataset(i).getXValue(a, j) < leftBound) {
                        leftBound = plot.getDataset(i).getXValue(a, j);
                    }
                    if (plot.getDataset(i).getXValue(a, j) > rightBound) {
                        rightBound = plot.getDataset(i).getXValue(a, j);
                    }
                }
            }
        }

        double lowerRange = 0;
        double upperRange = 0;

        if (lowerBound < leftBound) {
            lowerRange = lowerBound;
        } else {
            lowerRange = leftBound;
        }

        if (upperBound > rightBound) {
            upperRange = upperBound;
        } else {
            upperRange = rightBound;
        }

        NumberAxis domain = (NumberAxis) plot.getDomainAxis();
        domain.setRange(lowerRange - 20, upperRange + 20);
        domain.setVerticalTickLabels(true);
        NumberAxis range = (NumberAxis) plot.getRangeAxis();
        range.setRange(lowerRange - 20, upperRange + 20);
    }

    /**
     * Sizes the graph to an upper, lower, left, and right bound coordinate
     * @param lowerBound
     * @param upperBound
     * @param leftBound
     * @param rightBound
     */
    public void resizeGraph(double lowerBound, double upperBound, double leftBound, double rightBound) {
        NumberAxis domain = (NumberAxis) plot.getDomainAxis();
        domain.setRange(leftBound, rightBound);
        domain.setVerticalTickLabels(true);
        NumberAxis range = (NumberAxis) plot.getRangeAxis();
        range.setRange(lowerBound, upperBound);
    }

    /**
     * Scales the graph to a scale based on pixel size, each pixel representing <code>scale</code> units
     * @param scale
     */
    public void scaleGraphToScale(double scale, double centerX, double centerY) {
        double width = getWidth();
        double height = getHeight();

        double upperBound = centerY + ((height * scale) / 2);
        double lowerBound = centerY - ((height * scale) / 2);

        double leftBound = centerX - ((width  * scale) / 2);
        double rightBound = centerX + ((width  * scale) / 2);

        NumberAxis domain = (NumberAxis) plot.getDomainAxis();
        domain.setRange(leftBound, rightBound);
        domain.setVerticalTickLabels(true);
        NumberAxis range = (NumberAxis) plot.getRangeAxis();
        range.setRange(lowerBound, upperBound);
    }

    public void setGraphTheme(GraphTheme theme) {
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

    public JFreeChart getChart() {
        return chart;
    }

    public XYPlot getPlot() {
        return plot;
    }

    public ChartPanel getChartPanel() {
        return chartPanel;
    }
}