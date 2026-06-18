// MIT License
//
// Small command line utility for voxel-grid downsampling PCD files.

#include <pcl/PCLPointCloud2.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/io/pcd_io.h>

#include <filesystem>
#include <iomanip>
#include <iostream>
#include <sstream>
#include <stdexcept>
#include <string>

namespace {

struct Options {
    std::filesystem::path input_path;
    std::filesystem::path output_path;
    double leaf_size = 0.5;
    bool binary_compressed = false;
};

void PrintUsage(const char *argv0) {
    std::cerr << "Usage: " << argv0 << " INPUT_PCD [OUTPUT_PCD] [--leaf M] [--binary-compressed]\n"
              << "  INPUT_PCD            Input .pcd file path\n"
              << "  OUTPUT_PCD           Optional output path; defaults to INPUT_downsampled_LEAF.pcd\n"
              << "  --leaf M             Voxel leaf size in meters, default: 0.5\n"
              << "  --binary-compressed  Write binary-compressed PCD instead of binary PCD\n";
}

std::string LeafSizeToken(double leaf_size) {
    std::ostringstream stream;
    stream << std::fixed << std::setprecision(3) << leaf_size;
    std::string token = stream.str();
    while (!token.empty() && token.back() == '0') token.pop_back();
    if (!token.empty() && token.back() == '.') token.pop_back();
    return token.empty() ? "0" : token;
}

std::filesystem::path DefaultOutputPath(const std::filesystem::path &input_path, double leaf_size) {
    const auto parent = input_path.parent_path();
    const auto stem = input_path.stem().string();
    const auto extension = input_path.has_extension() ? input_path.extension().string() : ".pcd";
    return parent / (stem + "_downsampled_" + LeafSizeToken(leaf_size) + extension);
}

bool IsOption(const std::string &arg) { return arg.rfind("-", 0) == 0; }

Options ParseArgs(int argc, char **argv) {
    Options options;

    for (int i = 1; i < argc; ++i) {
        const std::string arg = argv[i];
        auto need_value = [&](const std::string &name) -> std::string {
            if (i + 1 >= argc) throw std::runtime_error("Missing value for " + name);
            return argv[++i];
        };

        if (arg == "--help" || arg == "-h") {
            PrintUsage(argv[0]);
            std::exit(0);
        } else if (arg == "--leaf") {
            options.leaf_size = std::stod(need_value(arg));
        } else if (arg == "--binary-compressed") {
            options.binary_compressed = true;
        } else if (IsOption(arg)) {
            throw std::runtime_error("Unknown option: " + arg);
        } else if (options.input_path.empty()) {
            options.input_path = arg;
        } else if (options.output_path.empty()) {
            options.output_path = arg;
        } else {
            throw std::runtime_error("Unexpected extra positional argument: " + arg);
        }
    }

    if (options.input_path.empty()) throw std::runtime_error("Missing INPUT_PCD");
    if (options.leaf_size <= 0.0) throw std::runtime_error("--leaf must be positive");
    if (options.output_path.empty()) options.output_path = DefaultOutputPath(options.input_path, options.leaf_size);
    return options;
}

}  // namespace

int main(int argc, char **argv) {
    try {
        const Options options = ParseArgs(argc, argv);

        pcl::PCLPointCloud2::Ptr input_cloud(new pcl::PCLPointCloud2);
        if (pcl::io::loadPCDFile(options.input_path.string(), *input_cloud) != 0) {
            std::cerr << "Failed to read PCD: " << options.input_path << "\n";
            return 1;
        }

        pcl::PCLPointCloud2 output_cloud;
        pcl::VoxelGrid<pcl::PCLPointCloud2> voxel_grid;
        voxel_grid.setInputCloud(input_cloud);
        voxel_grid.setLeafSize(static_cast<float>(options.leaf_size),
                               static_cast<float>(options.leaf_size),
                               static_cast<float>(options.leaf_size));
        voxel_grid.filter(output_cloud);

        if (!options.output_path.parent_path().empty()) {
            std::filesystem::create_directories(options.output_path.parent_path());
        }
        pcl::PCDWriter writer;
        const int save_result = options.binary_compressed
                                    ? writer.writeBinaryCompressed(options.output_path.string(), output_cloud)
                                    : writer.writeBinary(options.output_path.string(), output_cloud);
        if (save_result != 0) {
            std::cerr << "Failed to write PCD: " << options.output_path << "\n";
            return 1;
        }

        std::cout << "Input:  " << options.input_path << "\n"
                  << "Output: " << options.output_path << "\n"
                  << "Leaf:   " << options.leaf_size << "\n"
                  << "Points: " << input_cloud->width * input_cloud->height << " -> "
                  << output_cloud.width * output_cloud.height << "\n";
        return 0;
    } catch (const std::exception &e) {
        std::cerr << e.what() << "\n";
        PrintUsage(argv[0]);
        return 1;
    }
}
