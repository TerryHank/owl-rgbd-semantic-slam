#include <algorithm>
#include <chrono>
#include <cmath>
#include <csignal>
#include <map>
#include <memory>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <iterator>
#include <sstream>
#include <stdexcept>
#include <thread>
#include <vector>

#include <opencv2/dnn.hpp>
#include <opencv2/imgproc.hpp>
#include <rknn_api.h>

namespace {
using Clock = std::chrono::steady_clock;
constexpr int imageSize = 640, maskSize = 160, maskChannels = 32, anchors = 8400;
volatile std::sig_atomic_t stopping = 0;
void stop(int) { stopping = 1; }

void check(int code, const char* operation) {
    if (code < 0) throw std::runtime_error(std::string(operation) + " failed: " + std::to_string(code));
}

class Model {
public:
    explicit Model(char* path) { check(rknn_init(&context, path, 0, 0, nullptr), "rknn_init"); }
    ~Model() { rknn_destroy(context); }
    rknn_context context{};
};

void describe(rknn_context model) {
    rknn_sdk_version sdk{};
    check(rknn_query(model, RKNN_QUERY_SDK_VERSION, &sdk, sizeof(sdk)), "query SDK");
    std::cout << "rknn_api=" << sdk.api_version << " driver=" << sdk.drv_version << '\n';
    rknn_mem_size memory{};
    check(rknn_query(model, RKNN_QUERY_MEM_SIZE, &memory, sizeof(memory)), "query RKNN memory");
    std::cout << "npu_dma_bytes=" << memory.total_dma_allocated_size
              << " weights_bytes=" << memory.total_weight_size
              << " internal_bytes=" << memory.total_internal_size << '\n';
}

struct Output {
    explicit Output(rknn_context ctx) : context(ctx) {
        for (int i = 0; i < 2; ++i) {
            tensors[i].index = i;
            tensors[i].want_float = 1;
        }
        check(rknn_outputs_get(context, 2, tensors, nullptr), "rknn_outputs_get");
    }
    ~Output() { rknn_outputs_release(context, 2, tensors); }
    rknn_context context;
    rknn_output tensors[2]{};
};

int checkShape(rknn_context model) {
    rknn_tensor_attr predictions{};
    check(rknn_query(model, RKNN_QUERY_OUTPUT_ATTR, &predictions, sizeof(predictions)), "query predictions");
    const int classes = int(predictions.dims[1])-4-maskChannels;
    if (classes <= 0) throw std::runtime_error("Expected class scores before mask coefficients");
    const std::vector<std::vector<unsigned>> shapes = {
        {1, predictions.dims[1], anchors}, {1, maskChannels, maskSize, maskSize}};
    for (unsigned i = 0; i < shapes.size(); ++i) {
        rknn_tensor_attr tensor{};
        tensor.index = i;
        check(rknn_query(model, RKNN_QUERY_OUTPUT_ATTR, &tensor, sizeof(tensor)), "query output shape");
        if (tensor.n_dims != shapes[i].size() || !std::equal(shapes[i].begin(), shapes[i].end(), tensor.dims))
            throw std::runtime_error("Expected YOLO26n-seg 640 legacy ONNX output");
    }
    return classes;
}

struct Detection {
    int category, pixels;
    float score;
    std::vector<cv::Point3f> cameraPoints;
};

std::vector<Detection> instanceMasks(const Output& output, cv::Mat& unionMask, int classes,
                                     const cv::Mat& depth, const double* intrinsics, bool objects) {
    const auto* predictions = static_cast<const float*>(output.tensors[0].buf);
    const auto* prototypes = static_cast<const float*>(output.tensors[1].buf);
    std::vector<cv::Rect> boxes;
    std::vector<float> scores;
    std::vector<int> indices, categories;
    constexpr float threshold = .25f;
    const cv::Rect bounds(0, 0, imageSize, imageSize);
    for (int a = 0; a < anchors; ++a) {
        int category = 0;
        float score = predictions[4*anchors+a];
        for (int cls = 1; cls < classes; ++cls) {
            const float probability = predictions[(4+cls)*anchors+a];
            if (probability > score) { score = probability; category = cls; }
        }
        if (!std::isfinite(score) || score < threshold || (!objects && category != 0)) continue;
        const float x = predictions[a], y = predictions[anchors+a];
        const float w = predictions[2*anchors+a], h = predictions[3*anchors+a];
        cv::Rect box = cv::Rect(cvRound(x-w/2), cvRound(y-h/2), cvRound(w), cvRound(h)) & bounds;
        if (box.empty()) continue;
        boxes.push_back(box);
        scores.push_back(score);
        indices.push_back(a);
        categories.push_back(category);
    }
    std::vector<int> keep;
    std::map<int, std::vector<int>> byClass;
    for (int i = 0; i < int(boxes.size()); ++i) byClass[categories[i]].push_back(i);
    for (const auto& [category, group] : byClass) {
        std::vector<cv::Rect> candidates;
        std::vector<float> confidence;
        for (int index : group) {
            candidates.push_back(boxes[index]);
            confidence.push_back(scores[index]);
        }
        std::vector<int> selected;
        cv::dnn::NMSBoxes(candidates, confidence, threshold, .65f, selected);
        for (int i : selected) keep.push_back(group[i]);
    }
    unionMask = cv::Mat::zeros(maskSize, maskSize, CV_8U);
    std::vector<Detection> instances;
    for (int index : keep) {
        const auto box = boxes[index];
        const auto anchor = indices[index];
        Detection detection{categories[index], 0, scores[index], {}};
        const int pad = (imageSize-depth.rows)/2;
        for (int y = box.y/4; y < (box.y+box.height+3)/4; ++y) {
            auto* row = unionMask.ptr<unsigned char>(y);
            for (int x = box.x/4; x < (box.x+box.width+3)/4; ++x) {
                float logit = 0;
                for (int c = 0; c < maskChannels; ++c)
                    logit += predictions[(4+classes+c)*anchors+anchor] * prototypes[(c*maskSize+y)*maskSize+x];
                if (logit > 0.f) {
                    ++detection.pixels;
                    row[x] = 255;
                    const int u = x*4+2, v = y*4+2-pad;
                    if (depth.empty() || v < 0 || v >= depth.rows || (x%2 || y%2)) continue;
                    const float z = depth.at<float>(v, u);
                    if (std::isfinite(z) && z >= .3f && z <= 8.f)
                        detection.cameraPoints.emplace_back((u-intrinsics[2])*z/intrinsics[0],
                                                            (v-intrinsics[3])*z/intrinsics[1], z);
                }
            }
        }
        instances.push_back(std::move(detection));
    }
    return instances;
}
}

int main(int argc, char** argv) try {
    if (argc != 5 && argc != 6)
        throw std::runtime_error("Usage: yolo26-run MODEL.rknn RECORDING.owl|RGB640_LIST SECONDS FPS [OBJECTS.jsonl]");
    std::ifstream source(argv[2], std::ios::binary);
    if (!source) throw std::runtime_error("Cannot open input");
    const bool replay = std::filesystem::path(argv[2]).extension() == ".owl";
    int width = 0, height = 0;
    double intrinsics[4]{};
    const bool objects = argc == 6;
    std::ofstream observations;
    if (objects) {
        if (!replay) throw std::runtime_error("Object projection requires OWL1 RGB-D");
        observations.open(argv[5]);
        if (!observations) throw std::runtime_error("Cannot open instance output");
    }
    std::vector<std::vector<unsigned char>> frames;
    if (replay) {
        std::string header, tag;
        std::getline(source, header);
        std::istringstream fields(header);
        if (!(fields >> tag >> width >> height) || tag != "OWL1" || width != imageSize || height < 1 || height > imageSize)
            throw std::runtime_error("Expected an OWL1 recording with width 640 and height up to 640");
        for (double& value : intrinsics)
            if (!(fields >> value)) throw std::runtime_error("Missing OWL intrinsics");
    } else {
        for (std::string path; std::getline(source, path);) {
            std::ifstream image(path, std::ios::binary);
            std::vector<unsigned char> pixels((std::istreambuf_iterator<char>(image)), {});
            if (pixels.size() != imageSize*imageSize*3) throw std::runtime_error("Expected packed RGB640: " + path);
            frames.push_back(std::move(pixels));
        }
        if (frames.empty()) throw std::runtime_error("Empty image list");
    }
    std::vector<unsigned char> replayPixels(replay ? imageSize*imageSize*3 : 0, 114);
    const auto colorOffset = (imageSize-height)/2 * imageSize*3;
    const double seconds = std::stod(argv[3]), hz = std::stod(argv[4]);
    std::signal(SIGINT, stop);
    std::signal(SIGTERM, stop);
    cv::setNumThreads(1);
    Model model(argv[1]);
    check(rknn_set_core_mask(model.context, RKNN_NPU_CORE_0_1_2), "select RK3588 NPU cores");
    const int classes = checkShape(model.context);
    cv::Mat depth;
    if (objects) depth.create(height, width, CV_32FC1);
    describe(model.context);
    std::cout << "frame,input,people,mask_pixels_160,npu_ms,total_ms\n" << std::flush;
    const auto start = Clock::now();
    double origin = -1.;
    int cameraFrame = 0;
    for (size_t i = 0; !stopping && std::chrono::duration<double>(Clock::now()-start).count() < seconds; ++i) {
        const auto begin = Clock::now();
        size_t inputId;
        double captureTime = 0.;
        if (replay) {
            // Skip stale RGB-D frames and process the next frame at the recorded camera cadence.
            const double current = std::chrono::duration<double>(begin-start).count();
            std::string stamp;
            while (std::getline(source, stamp)) {
                captureTime = std::stod(stamp);
                if (origin < 0.) origin = captureTime;
                if (captureTime-origin >= current || i == 0) break;
                source.seekg(width*height*7, std::ios::cur);
                ++cameraFrame;
                stamp.clear();
            }
            if (stamp.empty() || !source) break;
            const auto scheduled = start + std::chrono::duration_cast<Clock::duration>(
                std::chrono::duration<double>(captureTime-origin));
            std::this_thread::sleep_until(scheduled);
            if (!source.read(reinterpret_cast<char*>(replayPixels.data()+colorOffset), width*height*3))
                throw std::runtime_error("Truncated OWL RGB frame");
            if (objects) {
                if (!source.read(reinterpret_cast<char*>(depth.data), width*height*4))
                    throw std::runtime_error("Truncated OWL depth frame");
            } else {
                source.seekg(width*height*4, std::ios::cur);
            }
            for (int j = colorOffset; j < colorOffset+width*height*3; j += 3)
                std::swap(replayPixels[j], replayPixels[j+2]);
            inputId = cameraFrame++;
        } else {
            inputId = i % frames.size();
        }
        const auto inferenceStart = Clock::now();
        auto& pixels = replay ? replayPixels : frames[inputId];
        rknn_input input{};
        input.buf = pixels.data();
        input.size = pixels.size();
        input.type = RKNN_TENSOR_UINT8;
        input.fmt = RKNN_TENSOR_NHWC;
        check(rknn_inputs_set(model.context, 1, &input), "rknn_inputs_set");
        check(rknn_run(model.context, nullptr), "rknn_run");
        Output output(model.context);
        rknn_perf_run perf{};
        check(rknn_query(model.context, RKNN_QUERY_PERF_RUN, &perf, sizeof(perf)), "query inference time");
        cv::Mat mask;
        const auto instances = instanceMasks(output, mask, classes, depth, intrinsics, objects);
        if (objects) {
            for (const auto& instance : instances) {
                if (instance.cameraPoints.size() < 4) continue;
                observations << std::fixed << std::setprecision(6) << "{\"stamp\":" << captureTime
                             << ",\"category\":" << instance.category << ",\"score\":" << instance.score
                             << ",\"points\":[";
                bool first = true;
                for (const auto& pt : instance.cameraPoints) {
                    if (!first) observations << ',';
                    first = false;
                    observations << '[' << pt.x << ',' << pt.y << ',' << pt.z << ']';
                }
                observations << "]}\n";
            }
            observations.flush();
        }
        if ((!replay && i < frames.size()) || (replay && i == 0)) {
            std::ofstream preview("mask-" + std::to_string(i) + ".pgm", std::ios::binary);
            preview << "P5\n160 160\n255\n";
            preview.write(reinterpret_cast<const char*>(mask.data), mask.total());
        }
        const auto duration = std::chrono::duration<double, std::milli>(Clock::now()-inferenceStart).count();
        std::cout << i << ',' << inputId << ',' << instances.size() << ',' << cv::countNonZero(mask)
                  << ',' << std::fixed << std::setprecision(3) << perf.run_duration/1000. << ',' << duration << std::endl;
        if (hz > 0.) std::this_thread::sleep_until(start +
            std::chrono::duration_cast<Clock::duration>(std::chrono::duration<double>((i+1)/hz)));
    }
    describe(model.context);
    return 0;
} catch (const std::exception& e) {
    std::cerr << e.what() << '\n';
    return 1;
}
