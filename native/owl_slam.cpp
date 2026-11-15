#include <algorithm>
#include <chrono>
#include <cmath>
#include <condition_variable>
#include <csignal>
#include <cstdint>
#include <cstdlib>
#include <deque>
#include <exception>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <map>
#include <memory>
#include <mutex>
#include <sstream>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

#include <opencv2/core.hpp>
#include <rtabmap/core/CameraModel.h>
#include <rtabmap/core/Memory.h>
#include <rtabmap/core/Odometry.h>
#include <rtabmap/core/OdometryInfo.h>
#include <rtabmap/core/Parameters.h>
#include <rtabmap/core/Rtabmap.h>
#include <rtabmap/core/SensorData.h>
#include <rtabmap/core/Signature.h>
#include <rtabmap/core/global_map/OccupancyGrid.h>

namespace {
volatile std::sig_atomic_t stopping = 0;
void stop(int) { stopping = 1; }

rtabmap::ParametersMap loadParameters(const char* path, rtabmap::ParametersMap* odometry = nullptr) {
    std::ifstream input(path);
    if (!input) throw std::runtime_error("Cannot open parameter file");
    rtabmap::ParametersMap params;
    auto* section = &params;
    std::string line, key, value, extra;
    while (std::getline(input, line)) {
        std::istringstream row(line);
        if (!(row >> key) || key[0] == '#') continue;
        if (key == "[odometry]" && odometry && !(row >> extra)) {
            section = odometry;
            continue;
        }
        if (!(row >> value) || (row >> extra) || !rtabmap::Parameters::getDefaultParameters().count(key))
            throw std::runtime_error("Invalid parameter line: " + line);
        (*section)[key] = value;
    }
    return params;
}

double elapsed(std::chrono::steady_clock::time_point start) {
    return std::chrono::duration<double, std::milli>(std::chrono::steady_clock::now()-start).count();
}

double percentile(std::vector<double>& times, double q) {
    if (times.empty()) return 0;
    std::sort(times.begin(), times.end());
    return times[static_cast<size_t>(std::ceil(q * times.size()))-1];
}

std::mutex outputMutex;

void logLine(const std::string& line) {
    const std::lock_guard<std::mutex> lock(outputMutex);
    std::cout << line << '\n';
}

struct ColoredPoint {
    float x, y, z;
    std::uint8_t r, g, b, a;
};
static_assert(sizeof(ColoredPoint) == 16);

ColoredPoint transformPoint(const rtabmap::Transform& t, const ColoredPoint& p) {
    return {t.r11()*p.x + t.r12()*p.y + t.r13()*p.z + t.x(),
            t.r21()*p.x + t.r22()*p.y + t.r23()*p.z + t.y(),
            t.r31()*p.x + t.r32()*p.y + t.r33()*p.z + t.z(), p.r, p.g, p.b, p.a};
}

class LiveMap {
public:
    LiveMap(const char* path, const rtabmap::ParametersMap& params)
        : path_(path), temporary_(path_ + ".tmp"), grid_(&grids_, params),
          writeGraph_(std::getenv("OWL_SEMANTIC_GRAPH") && *std::getenv("OWL_SEMANTIC_GRAPH")) {}

    void update(const rtabmap::Rtabmap& slam, const rtabmap::CameraModel& camera,
                const cv::Mat& color, const cv::Mat& depth, int frame, int tracked,
                int revision, double stamp, int loop, double slamMs) {
        const auto started = std::chrono::steady_clock::now();
        const auto& poses = slam.getLocalOptimizedPoses();
        if (poses.empty()) return;
        const auto* keyframe = slam.getMemory()->getLastWorkingSignature(true);
        if (keyframe && !clouds_.count(keyframe->id())) {
            auto& points = clouds_[keyframe->id()];
            // Sample each keyframe once in robot coordinates; use fresh optimized poses to assemble.
            for (int v = 0; v < depth.rows; v += 8) {
                for (int u = 0; u < depth.cols; u += 8) {
                    const float z = depth.at<float>(v, u);
                    if (z == 0.f) continue;
                    const auto bgr = color.at<cv::Vec3b>(v, u);
                    points.push_back(transformPoint(camera.localTransform(), {
                        float((u-camera.cx())*z/camera.fx()), float((v-camera.cy())*z/camera.fy()),
                        z, bgr[2], bgr[1], bgr[0], 255}));
                }
            }
        }
        double moved = 0., rotated = 0.;
        std::vector<ColoredPoint> cloud;
        for (const auto& [id, pose] : poses) {
            if (!grids_.localGrids().count(id)) {
                const auto* signature = slam.getMemory()->getSignature(id);
                if (signature && signature->sensorData().gridCellSize() > 0.f) {
                    const auto& sensor = signature->sensorData();
                    cv::Mat ground, obstacles, empty;
                    sensor.uncompressDataConst(nullptr, nullptr, nullptr, nullptr, &ground, &obstacles, &empty);
                    grids_.add(id, ground, obstacles, empty, sensor.gridCellSize(), sensor.gridViewPoint());
                }
            }
            if (const auto previous = previous_.find(id); previous != previous_.end()) {
                moved = std::max(moved, double(pose.getDistance(previous->second)));
                rotated = std::max(rotated, std::abs(std::remainder(double(pose.theta()-previous->second.theta()),
                                                                    2*std::acos(-1.))) * 180/std::acos(-1.));
            }
            if (const auto points = clouds_.find(id); points != clouds_.end()) {
                for (const auto& p : points->second) cloud.push_back(transformPoint(pose, p));
            }
        }
        if (loop > 0) grid_.clear();
        grid_.update(poses);
        float x, y;
        const auto cells = grid_.getMap(x, y);
        if (cells.empty() || cloud.empty()) return;
        previous_ = poses;

        // Commit the grid and cloud together, so the viewer can only read complete revisions.
        std::ofstream output(temporary_, std::ios::binary | std::ios::trunc);
        output << std::fixed << std::setprecision(6)
               << "OWL_MAP1 " << revision << ' ' << stamp << ' ' << frame << ' ' << tracked << ' '
               << poses.size() << ' ' << loop << ' ' << moved << ' ' << rotated << ' '
               << cells.cols << ' ' << cells.rows << ' ' << x << ' ' << y << ' ' << grid_.getCellSize()
               << ' ' << cloud.size() << ' ' << slamMs << ' ' << elapsed(started) << '\n';
        for (int row = 0; row < cells.rows; ++row)
            output.write(cells.ptr<char>(row), cells.cols);
        output.write(reinterpret_cast<const char*>(cloud.data()), cloud.size()*sizeof(ColoredPoint));
        output.close();
        if (!output) throw std::runtime_error("Cannot write live map");
        if (writeGraph_) {
            std::ofstream graph(temporary_+".poses");
            graph << "revision,stamp,ox,oy,oyaw,x,y,z,yaw\n" << std::fixed << std::setprecision(8);
            for (const auto& [id, optimized] : poses) {
                const auto* signature = slam.getMemory()->getSignature(id);
                if (!signature) continue;
                const auto& raw = signature->getPose();
                graph << revision << ',' << signature->getStamp() << ',' << raw.x() << ',' << raw.y()
                      << ',' << raw.theta() << ',' << optimized.x() << ',' << optimized.y() << ','
                      << optimized.z() << ',' << optimized.theta() << '\n';
            }
            graph.close();
            if (!graph) throw std::runtime_error("Cannot write semantic graph poses");
            std::filesystem::rename(temporary_+".poses", path_+".poses.csv");
        }
        std::filesystem::rename(temporary_, path_);
        if (loop > 0) {
            std::ostringstream message;
            message << "live_loop revision=" << revision << " nodes=" << poses.size()
                    << " shift_m=" << moved << " yaw_deg=" << rotated;
            logLine(message.str());
        }
    }

private:
    std::string path_, temporary_;
    rtabmap::LocalGridCache grids_;
    rtabmap::OccupancyGrid grid_;
    std::map<int, std::vector<ColoredPoint>> clouds_;
    std::map<int, rtabmap::Transform> previous_;
    bool writeGraph_;
};

using Clock = std::chrono::steady_clock;

struct MappingFrame {
    rtabmap::SensorData data;
    rtabmap::Transform pose;
    cv::Mat covariance;
    cv::Mat color, depth;
    int tracked;
    Clock::time_point submitted;
};

class MappingWorker {
public:
    MappingWorker(const rtabmap::ParametersMap& params, const char* database, const char* livePath,
                  const rtabmap::CameraModel& camera)
        : camera_(camera) {
        slam_.init(params, database);
        if (livePath) live_ = std::make_unique<LiveMap>(livePath, params);
        thread_ = std::thread(&MappingWorker::run, this);
    }

    ~MappingWorker() { join(); }

    void submit(MappingFrame frame) {
        std::unique_lock<std::mutex> lock(mutex_);
        // Keep every selected keyframe and bound memory if the backend falls behind.
        changed_.wait(lock, [&] { return error_ || pending_.size() < 4; });
        if (error_) std::rethrow_exception(error_);
        pending_.push_back(std::move(frame));
        peakPending_ = std::max(peakPending_, pending_.size());
        changed_.notify_all();
    }

    rtabmap::Transform correction() {
        const std::lock_guard<std::mutex> lock(mutex_);
        if (error_) std::rethrow_exception(error_);
        return correction_;
    }

    void finish() {
        join();
        if (error_) std::rethrow_exception(error_);
    }

    void report() {
        std::cout << " map_updates=" << mapped_ << " graph_nodes=" << nodes_ << " closures=" << loops_
                  << " map_p95_ms=" << percentile(mapTimes_, .95)
                  << " map_queue_p95_ms=" << percentile(queueTimes_, .95)
                  << " map_queue_max_ms=" << (queueTimes_.empty() ? 0. : queueTimes_.back())
                  << " map_queue_peak=" << peakPending_;
    }

    int updates() const { return mapped_; }

private:
    void join() {
        if (!thread_.joinable()) return;
        {
            const std::lock_guard<std::mutex> lock(mutex_);
            done_ = true;
        }
        changed_.notify_all();
        thread_.join();
    }

    void run() {
        try {
            for (;;) {
                std::unique_lock<std::mutex> lock(mutex_);
                changed_.wait(lock, [&] { return done_ || !pending_.empty(); });
                if (pending_.empty()) break;
                auto frame = std::move(pending_.front());
                pending_.pop_front();
                lock.unlock();
                changed_.notify_all();
                process(frame);
            }
            nodes_ = slam_.getLocalOptimizedPoses().size();
            if (const char* path = std::getenv("OWL_KEYFRAME_POSES"); path && *path) {
                std::ofstream output(path);
                output << "node_id,x,y,z,yaw\n" << std::fixed << std::setprecision(7);
                for (const auto& [id, pose] : slam_.getLocalOptimizedPoses())
                    output << id << ',' << pose.x() << ',' << pose.y() << ',' << pose.z() << ',' << pose.theta() << '\n';
                if (!output) throw std::runtime_error("Cannot export optimized keyframe poses");
            }
            slam_.close();
        } catch (...) {
            {
                const std::lock_guard<std::mutex> lock(mutex_);
                error_ = std::current_exception();
            }
            changed_.notify_all();
        }
    }

    void process(const MappingFrame& frame) {
        const auto started = Clock::now();
        queueTimes_.push_back(elapsed(frame.submitted));
        if (slam_.process(frame.data, frame.pose, frame.covariance)) {
            const auto optimized = slam_.getMapCorrection().clone();
            {
                const std::lock_guard<std::mutex> lock(mutex_);
                correction_ = optimized;
            }
            ++mapped_;
            const int target = slam_.getLoopClosureId();
            if (live_) live_->update(slam_, camera_, frame.color, frame.depth, frame.data.id(), frame.tracked,
                                     mapped_, frame.data.stamp(), target, elapsed(started));
            if (target > 0) {
                ++loops_;
                std::ostringstream message;
                message << "loop " << frame.data.id() << " -> " << target
                        << " correction=" << optimized.prettyPrint()
                        << " backend_ms=" << elapsed(started) << " map_delay_ms=" << elapsed(frame.submitted);
                logLine(message.str());
            }
        }
        mapTimes_.push_back(elapsed(started));
    }

    const rtabmap::CameraModel& camera_;
    rtabmap::Rtabmap slam_;
    std::unique_ptr<LiveMap> live_;
    std::mutex mutex_;
    std::condition_variable changed_;
    std::deque<MappingFrame> pending_;
    std::exception_ptr error_;
    rtabmap::Transform correction_ = rtabmap::Transform::getIdentity();
    std::thread thread_;
    bool done_ = false;
    int mapped_ = 0, loops_ = 0;
    size_t nodes_ = 0, peakPending_ = 0;
    std::vector<double> mapTimes_, queueTimes_;
};

class GyroHeading {
public:
    explicit GyroHeading(const char* path) {
        std::ifstream input(path);
        std::string magic;
        double x, y, z, roll, pitch, yaw;
        if (!(input >> magic >> x >> y >> z >> roll >> pitch >> yaw) || magic != "IMU1" ||
            !std::isfinite(x+y+z+roll+pitch+yaw))
            throw std::runtime_error("Expected IMU1 calibration header for OWL_GYRO");
        const rtabmap::Transform base(x,y,z,roll,pitch,yaw);
        double stamp;
        while (input >> stamp) {
            double wx, wy, wz, ax, ay, az;
            if (!(input >> wx >> wy >> wz >> ax >> ay >> az) || !std::isfinite(stamp+wx+wy+wz) ||
                (!samples_.empty() && stamp <= samples_.back().stamp))
                throw std::runtime_error("Invalid or unordered gyroscope measurements");
            Sample next{stamp, base.r31()*wx + base.r32()*wy + base.r33()*wz, 0., 0};
            if (!samples_.empty()) {
                const auto& last = samples_.back();
                const double dt = stamp-last.stamp;
                next.angle = last.angle + .5*dt*(last.rate+next.rate);
                next.gaps = last.gaps + (dt > .05);
            }
            samples_.push_back(next);
        }
        if (!input.eof() || samples_.size() < 2)
            throw std::runtime_error("Incomplete gyroscope recording");
    }

    rtabmap::Transform guess(const rtabmap::Odometry& odometry, double stamp) {
        if (!odometry.framesProcessed()) return {};
        const double previous = odometry.previousStamp();
        if (previous < samples_.front().stamp || stamp > samples_.back().stamp || stamp <= previous) {
            ++uncovered_;
            return {};
        }
        const auto from = at(previous), to = at(stamp);
        if (from.gaps != to.gaps || from.gaps < 0 || to.gaps < 0) {
            ++uncovered_;
            return {};
        }
        const auto& velocity = odometry.getVelocityGuess();
        const double dt = stamp-previous;
        ++used_;
        return rtabmap::Transform(velocity.isNull() ? 0. : velocity.x()*dt,
                                  velocity.isNull() ? 0. : velocity.y()*dt, 0.,
                                  0., 0., std::remainder(to.angle-from.angle, 2*std::acos(-1.)));
    }

    size_t used() const { return used_; }
    size_t uncovered() const { return uncovered_; }

private:
    struct Sample { double stamp, rate, angle; int gaps; };

    Sample at(double stamp) const {
        const auto right = std::lower_bound(samples_.begin(), samples_.end(), stamp,
            [](const Sample& sample, double time) { return sample.stamp < time; });
        if (right == samples_.begin() || right->stamp == stamp) return *right;
        const auto& left = *(right-1);
        const double span = right->stamp-left.stamp, delta = stamp-left.stamp;
        const double rate = left.rate + (right->rate-left.rate)*delta/span;
        return {stamp, rate, left.angle+.5*delta*(left.rate+rate), span > .05 ? -1 : left.gaps};
    }

    std::vector<Sample> samples_;
    size_t used_ = 0, uncovered_ = 0;
};
}

int main(int argc, char** argv) try {
    if ((argc != 3 && argc != 4) || std::filesystem::exists(argv[2])) {
        std::cerr << "Usage: owl_slam owlslam.params NEW_MAP.db [LIVE_MAP] < owl_stream\n";
        return 2;
    }
    rtabmap::ParametersMap frontend;
    const auto params = loadParameters(argv[1], &frontend);
    const double mappingHz = std::stod(params.at("Rtabmap/DetectionRate"));
    std::signal(SIGINT, stop);
    std::signal(SIGTERM, stop);
    cv::setNumThreads(2);
    std::cin.sync_with_stdio(false);
    std::cout << std::fixed << std::setprecision(6);

    // Header describes a rectified, registered pixel grid and the optical-to-base transform.
    std::string line, magic, extra;
    int width, height;
    double fx, fy, cx, cy, x, y, z, roll, pitch, yaw;
    if (!std::getline(std::cin, line)) throw std::runtime_error("Missing OWL stream header");
    std::istringstream header(line);
    if (!(header >> magic >> width >> height >> fx >> fy >> cx >> cy >> x >> y >> z >> roll >> pitch >> yaw)
        || magic != "OWL1" || (header >> extra) || width <= 0 || height <= 0 ||
        width > 4096 || height > 4096 || fx <= 0 || fy <= 0 ||
        !std::isfinite(fx+fy+cx+cy+x+y+z+roll+pitch+yaw)) {
        throw std::runtime_error("Invalid OWL stream header");
    }
    const rtabmap::CameraModel camera(fx, fy, cx, cy, rtabmap::Transform(x,y,z,roll,pitch,yaw),
                                     0., cv::Size(width,height));
    auto odomParams = params;
    for (const auto& [name, value] : frontend) odomParams[name] = value;
    if (const char* overrides = std::getenv("OWL_ODOM_PARAMS"); overrides && *overrides) {
        for (const auto& [name, value] : loadParameters(overrides)) odomParams[name] = value;
    }
    std::unique_ptr<rtabmap::Odometry> odom(rtabmap::Odometry::create(odomParams));
    std::unique_ptr<GyroHeading> gyroHeading;
    if (const char* path = std::getenv("OWL_GYRO"); path && *path) {
        if (odom->getType() != rtabmap::Odometry::kTypeF2M)
            throw std::runtime_error("OWL_GYRO is an optional heading prior for F2M");
        gyroHeading = std::make_unique<GyroHeading>(path);
    }
    MappingWorker mapper(params, argv[2], argc == 4 ? argv[3] : nullptr, camera);
    int frames = 0, tracked = 0;
    std::ofstream rawPoses;
    if (const char* path = std::getenv("OWL_RAW_POSES"); path && *path) {
        rawPoses.open(path);
        if (!rawPoses) throw std::runtime_error("Cannot open semantic odometry file");
        rawPoses << "stamp,x,y,z,yaw\n" << std::fixed << std::setprecision(8);
    }
    double lastStamp = -1, lastMap = -1, firstStamp = -1;
    std::vector<double> odomTimes;
    const auto runStart = Clock::now();
    auto firstArrival = runStart;
    const bool benchmark = std::getenv("OWL_BENCH") != nullptr;
    while (!stopping && std::getline(std::cin, line)) {
        double stamp;
        std::istringstream record(line);
        if (!(record >> stamp) || (record >> extra) || !std::isfinite(stamp) || stamp <= lastStamp)
            throw std::runtime_error("Expected increasing capture timestamps");
        lastStamp = stamp;
        const auto frameStart = Clock::now();
        if (firstStamp < 0.) {
            firstStamp = stamp;
            firstArrival = frameStart;
        }
        cv::Mat color(height, width, CV_8UC3), depth(height, width, CV_32FC1);
        if (!std::cin.read(reinterpret_cast<char*>(color.data), color.total()*color.elemSize()) ||
            !std::cin.read(reinterpret_cast<char*>(depth.data), depth.total()*depth.elemSize())) {
            if (stopping) break;
            throw std::runtime_error("Truncated RGB-D frame");
        }
        for (auto& value : cv::Mat_<float>(depth))
            if (!std::isfinite(value) || value < .3f || value > 8.f) value = 0.f;
        rtabmap::SensorData data(color, depth, camera, ++frames, stamp);
        rtabmap::OdometryInfo info;
        const auto started = Clock::now();
        // The gyroscope seeds registration. RGB-D still estimates the final relative pose.
        const auto pose = gyroHeading ? odom->process(data, gyroHeading->guess(*odom, stamp), &info)
                                      : odom->process(data, &info);
        odomTimes.push_back(elapsed(started));
        if (!pose.isNull()) {
            ++tracked;
            if (rawPoses.is_open())
                rawPoses << stamp << ',' << pose.x() << ',' << pose.y() << ',' << pose.z() << ','
                         << pose.theta() << std::endl;
            if (lastMap < 0 || mappingHz <= 0 || stamp-lastMap+1e-6 >= 1./mappingHz) {
                lastMap = stamp;
                mapper.submit({std::move(data), pose.clone(), info.reg.covariance.clone(),
                               color, depth, tracked, Clock::now()});
            }
        }
        const auto correction = mapper.correction();
        std::ostringstream message;
        message << std::fixed << std::setprecision(6);
        if (benchmark) {
            message << "bench " << frames << ' ' << stamp << ' ' << elapsed(frameStart)
                    << ' ' << elapsed(firstArrival)-(stamp-firstStamp)*1000 << " 0";
            logLine(message.str());
            message.str("");
        }
        if (!pose.isNull()) {
            message << "pose " << stamp << ' ' << (correction*pose).prettyPrint();
            logLine(message.str());
        }
    }
    const double frontendMs = elapsed(runStart);
    mapper.finish();
    const double odomP95 = percentile(odomTimes, .95);
    const double odomMax = odomTimes.empty() ? 0 : odomTimes.back();
    std::cout << "summary frames=" << frames << " tracked=" << tracked
              << " odom_p95_ms=" << odomP95 << " odom_max_ms=" << odomMax;
    mapper.report();
    if (gyroHeading)
        std::cout << " gyro_guesses=" << gyroHeading->used() << " gyro_uncovered=" << gyroHeading->uncovered();
    std::cout << " frontend_ms=" << frontendMs
              << " wall_ms=" << elapsed(runStart) << '\n';
    return mapper.updates() > 0 ? 0 : 1;
} catch (const std::exception& error) {
    std::cerr << error.what() << '\n';
    return 1;
}
