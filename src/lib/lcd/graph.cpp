#include "lib/lcd/graph.hpp"
#include "robotiO.hpp"
#include "main.h"

namespace lcd {
namespace graph {
int yMin = 0;
int yMax = 1;
int points = 10;
lv_obj_t* graph;
lv_chart_series_t* position;

void init() {
  graph = lv_chart_create(flywheelTab, NULL);
  lv_obj_set_size(graph, 450, 230);
  lv_obj_align(graph, NULL, LV_ALIGN_CENTER, 0, 0);
  lv_chart_set_type(graph, LV_CHART_TYPE_LINE);
  lv_chart_set_series_width(graph, 5);

  lv_chart_set_range(graph, 0, 1);

  position = lv_chart_add_series(graph, LV_COLOR_RED);
  lv_chart_set_point_count(graph, points);
}

void display() {
  lv_chart_set_range(graph, yMin, yMax);
  lv_chart_refresh(graph);
}

/**
 * int data the data to be plotted, takes Y value
 * int y the amount to
 */
void addData(int data) {
  if (data > yMax)
    yMax = data;
  else if (data < yMin) {
    yMin = data;
  }
  points++;
  //lv_chart_set_point_count(graph, points);
  lv_chart_set_range(graph, yMin, yMax);
  lv_chart_set_next(graph, position, data);
  lv_chart_refresh(graph);
}
}  // namespace graph
}  // namespace lcd
