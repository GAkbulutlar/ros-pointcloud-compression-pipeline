#include "BagPointCloudPipeline.h"

#include <cstdlib>
#include <iostream>
#include <string>

namespace {

void printUsage(const char* exe)
{
    std::cout << "Usage: " << exe
              << " [--bag <bag_path>] [--out <output_bin>] [--max-frames <n>]"
              << " [--radius <meters>] [--voxel-leaf <meters>] [--no-view] [--no-plots]\n";
}

bool parseArgs(int argc, char** argv, PipelineConfig& config)
{
    for (int i = 1; i < argc; ++i) {
        const std::string arg = argv[i];

        auto requireValue = [&](const char* flag) -> const char* {
            if (i + 1 >= argc) {
                std::cerr << "Missing value for " << flag << "\n";
                return nullptr;
            }
            return argv[++i];
        };

        if (arg == "--bag") {
            const char* value = requireValue("--bag");
            if (!value) {
                return false;
            }
            config.bag_path = value;
        } else if (arg == "--out") {
            const char* value = requireValue("--out");
            if (!value) {
                return false;
            }
            config.output_bin_path = value;
        } else if (arg == "--max-frames") {
            const char* value = requireValue("--max-frames");
            if (!value) {
                return false;
            }
            config.max_frames = static_cast<std::size_t>(std::strtoul(value, nullptr, 10));
        } else if (arg == "--radius") {
            const char* value = requireValue("--radius");
            if (!value) {
                return false;
            }
            config.kd_query_radius = static_cast<float>(std::atof(value));
        } else if (arg == "--voxel-leaf") {
            const char* value = requireValue("--voxel-leaf");
            if (!value) {
                return false;
            }
            config.voxel_leaf_size = static_cast<float>(std::atof(value));
        } else if (arg == "--no-view") {
            config.enable_visualization = false;
        } else if (arg == "--no-plots") {
            config.enable_metric_plots = false;
        } else if (arg == "--help" || arg == "-h") {
            printUsage(argv[0]);
            return false;
        } else {
            std::cerr << "Unknown argument: " << arg << "\n";
            printUsage(argv[0]);
            return false;
        }
    }

    return true;
}

} // namespace

int main(int argc, char** argv)
{
    PipelineConfig config;

    try {
        if (!parseArgs(argc, argv, config)) {
            return 1;
        }

        BagPointCloudPipeline pipeline(config);
        if (!pipeline.run()) {
            return 1;
        }
    } catch (const std::exception& e) {
        std::cerr << "Fatal error: " << e.what() << "\n";
        return 1;
    } catch (...) {
        std::cerr << "Fatal error: unknown exception\n";
        return 1;
    }

    return 0;
}
