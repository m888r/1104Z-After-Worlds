#include "lib/path/pathgroup.hpp"

namespace path {
PathGroup::PathGroup(std::initializer_list<std::reference_wrapper<Path>> list, int resolution, int lookahead)
    : Path::Path(resolution, lookahead)
    , paths(list) {}

Point PathGroup::pointAt(int t) {
    // TODO: Make it optionally max the point at the end of each path so it doesn't get trolled
    int runningSum = 0;
    Point point;
    for (Path& path : paths) {
        int lastRunningSum = runningSum;
        runningSum += path.getResolution();
        if (t <= runningSum) {
            point = path.pointAt(t - lastRunningSum);
            point.t = t;
            return point;
        }
        point = path.pointAt(path.getResolution());
        point.t = t; //change its t to be not monkey
    }
    return point;
}
} // namespace path