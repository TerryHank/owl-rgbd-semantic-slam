#include <fstream>
#include <iostream>
#include <memory>
#include <string>
#include <opencv2/imgcodecs.hpp>
#include <rtabmap/core/DBDriver.h>
#include <rtabmap/core/LocalGridMaker.h>
#include <rtabmap/core/global_map/OccupancyGrid.h>

int main(int argc, char **argv) {
  if (argc < 3) {
    std::cerr << "Usage: rebuild_owl_grid COPY.db output_prefix [--Grid/... value ...]\n";
    return 2;
  }
  const auto params = rtabmap::Parameters::parseArguments(argc, argv);
  const bool rebuild = !params.empty();
  std::unique_ptr<rtabmap::DBDriver> db(rtabmap::DBDriver::create());
  if (!db->openConnection(argv[1], false, !rebuild)) return 1;
  const auto poses = db->loadOptimizedPoses();
  if (poses.empty()) return 1;
  rtabmap::LocalGridCache cache;
  rtabmap::LocalGridMaker maker(params);
  for (const auto &[id, pose] : poses) {
    if (id < 1 || pose.isNull()) continue;
    rtabmap::SensorData data;
    db->getNodeData(id, data, rebuild, false, false, !rebuild);
    data.uncompressData();
    if (rebuild) {
      cv::Mat ground, obstacles, empty;
      cv::Point3f viewpoint;
      rtabmap::Signature node(data);
      node.setPose(pose);
      maker.createLocalMap(node, ground, obstacles, empty, viewpoint);
      cache.add(id, ground, obstacles, empty, maker.getCellSize(), viewpoint);
      db->updateOccupancyGrid(id, ground, obstacles, empty, maker.getCellSize(), viewpoint);
    } else {
      cache.add(id, data.gridGroundCellsRaw(), data.gridObstacleCellsRaw(),
                data.gridEmptyCellsRaw(), data.gridCellSize(), data.gridViewPoint());
    }
  }
  rtabmap::OccupancyGrid grid(&cache, params);
  grid.update(poses);
  float x = 0, y = 0;
  const cv::Mat map = grid.getMap(x, y);
  if (map.empty()) return 1;
  if (rebuild) db->save2DMap(map, x, y, maker.getCellSize());
  cv::Mat pgm(map.size(), CV_8U);
  int free = 0, occupied = 0;
  for (int row = 0; row < map.rows; ++row) {
    for (int col = 0; col < map.cols; ++col) {
      const int cell = map.at<signed char>(row, col);
      free += cell == 0;
      occupied += cell == 100;
      pgm.at<unsigned char>(map.rows-1-row,col) = cell < 0 ? 205 : (cell == 0 ? 254 : 0);
    }
  }
  const std::string prefix(argv[2]);
  if (!cv::imwrite(prefix + ".pgm", pgm)) return 1;
  std::ofstream metadata(prefix + ".yaml");
  metadata << "image: " << prefix.substr(prefix.find_last_of('/')+1) << ".pgm\n"
           << "resolution: " << maker.getCellSize() << "\norigin: [" << x << ", " << y
           << ", 0]\nnegate: 0\noccupied_thresh: 0.65\nfree_thresh: 0.196\n";
  std::cout << poses.size() << " poses, " << map.cols << "x" << map.rows
            << ", free=" << free << ", occupied=" << occupied << '\n';
}
