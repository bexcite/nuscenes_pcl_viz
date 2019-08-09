#include <iostream>
#include <fstream>
#include <map>

#include "jsoncpp/json/json.h"
#include "yaml-cpp/yaml.h"

#include "pcl/point_types.h"
#include "pcl/point_cloud.h"
#include "pcl/visualization/pcl_visualizer.h"

void PrintUsage() {
    std::cout << "Usage: nuscenes_pcl_viz <config_file>\n";
    std::cout << "\n  Reads scene according to config file and visualize it in PCLVisualizer\n";
}

struct NuScenesConfig {
    std::string name;
    std::string dir;
    Json::Value scene;
    Json::Value sample;
    Json::Value sampleData;

    Json::Value nullValue;


    struct LidarSweep {
        uint64_t timestamp;
        std::string filename;
        void Print(std::ostream& os = std::cout) {
            os << "LidarSweep: timestamp = " << timestamp << ", filename = " << filename << std::endl;
        }
    };


    Json::Value& GetSampleById(const std::string sample_id) {
        Json::Value::iterator it = std::find_if(sample.begin(), sample.end(), [&sample_id](Json::Value& v) {
            return v["token"] == sample_id;
        });
        if (it != sample.end()) {
            return *it;
        }
        return nullValue;
    }


    Json::Value& GetNextSample(const Json::Value& sample) {
        if (sample.isNull()) {
            return nullValue;
        }
        return GetSampleById(sample["next"].asString());
    }


    Json::Value GetLidarSamples(const Json::Value& sample) {
        std::map<uint64_t, Json::Value> m;
        std::string sample_token = sample["token"].asString();
        for (const Json::Value& data : sampleData) {
            if (data["sample_token"].asString() == sample_token && 
                    data["filename"].asString().find("LIDAR_TOP") != std::string::npos) {
                m[data["timestamp"].asUInt64()] = data;
            }
        }
        Json::Value lidarSamples = Json::Value(Json::arrayValue);
        for (auto v : m) {
            lidarSamples.append(v.second);
        }
        return lidarSamples;
    }


    std::vector<LidarSweep> GetSceneSweeps(const Json::Value& scene) {
        std::vector<LidarSweep> sweeps;
        std::string first_sample_token = scene["first_sample_token"].asString();
        std::string last_sample_token = scene["last_sample_token"].asString();
        
        Json::Value& sample = GetSampleById(first_sample_token);
        while (!sample.isNull()) {
            auto lidarSamples = GetLidarSamples(sample);
            for (auto& ls : lidarSamples) {
                sweeps.push_back(LidarSweep{
                    ls["timestamp"].asUInt64(),
                    dir + "/" + ls["filename"].asString()
                });
            }
            if (sample["token"].asString() == last_sample_token) {
                break;
            } 
            sample = GetNextSample(sample);
        }
        return sweeps;
    }


    void Print(std::ostream& os = std::cout) {
        os << "NuScenesConfig: name = " << name << ", dir = " << dir << std::endl
           << "  scene.size = " << scene.size() << std::endl
           << "  sample.size = " << sample.size() << std::endl
           << "  sampleData.size = " << sampleData.size() << std::endl;
    }


    std::string DataPath() {return dir + "/" + name; }


    static 
    bool Reader(const std::string& config_dir, NuScenesConfig& nuScenesConfig) {
        YAML::Node config = YAML::LoadFile(config_dir);
        nuScenesConfig.name = config["nu_scenes_dataset"].as<std::string>();
        nuScenesConfig.dir = config["nu_scenes_dir"].as<std::string>();

        std::ifstream configScene(nuScenesConfig.DataPath() + "/scene.json");
        if (!configScene.good()) {
            std::cerr << "Can't open file: " << nuScenesConfig.DataPath() + "/scene.json" << std::endl;
            return false;
        }

        std::ifstream configSample(nuScenesConfig.DataPath() + "/sample.json");
        if (!configSample.good()) {
            std::cerr << "Can't open file: " << nuScenesConfig.DataPath() + "/sample.json" << std::endl;
            return false;
        }

        std::ifstream configSampleData(nuScenesConfig.DataPath() + "/sample_data.json");
        if (!configSampleData.good()) {
            std::cerr << "Can't open file: " << nuScenesConfig.DataPath() + "/sample_data.json" << std::endl;
            return false;
        }

        Json::Reader reader;
        reader.parse(configScene,      nuScenesConfig.scene);
        reader.parse(configSample,     nuScenesConfig.sample);
        reader.parse(configSampleData, nuScenesConfig.sampleData);
        return true;
    }

};


bool ReadPCDBin(const std::string& fname, pcl::PointCloud<pcl::PointXYZI>& cloud) {
    if (fname.empty()) return false;
    std::ifstream file(fname, std::ios::in | std::ios::binary);
    if (!file.good()) {
        std::cerr << "Error during openning the file: " << fname << std::endl;
        return false;
    }

    int cnt = 0;
    float max_i = 0;
    while (file) {
        float x,y,z,i,r;
        file.read(reinterpret_cast<char*>(&x), sizeof(float));
        file.read(reinterpret_cast<char*>(&y), sizeof(float));
        file.read(reinterpret_cast<char*>(&z), sizeof(float));
        file.read(reinterpret_cast<char*>(&i), sizeof(float));
        file.read(reinterpret_cast<char*>(&r), sizeof(float));
        max_i = std::max(max_i, i);
        pcl::PointXYZI point{};
        point.x = x;
        point.y = y;
        point.z = z;
        point.intensity = i; //std::sqrt(x*x + y*y + z*z) < 10 ? 3 : 250 ;
        cloud.push_back(point);
    }
    file.close();
    return true;
}

bool ReadPCDBin(const std::string& fname, const pcl::PointCloud<pcl::PointXYZI>::Ptr& cloud) {
    return ReadPCDBin(fname, *cloud);
}


int main(int argc, char* argv[]) {
    std::cout << "NuScenes Lidar Sweeps PCL Visualization.\n";

    if (argc != 2) {
        std::cerr << "Error: need config file.\n";
        PrintUsage();
        return EXIT_FAILURE;
    }

    NuScenesConfig nuScenesConfig;
    if (!NuScenesConfig::Reader(argv[1], nuScenesConfig)) {
        return EXIT_FAILURE;
    }

    nuScenesConfig.Print();

    std::vector<std::vector<NuScenesConfig::LidarSweep>> sceneSweeps;

    std::cout << "Got scenes:\n";
    for (Json::Value& scene : nuScenesConfig.scene) {
        std::vector<NuScenesConfig::LidarSweep> sweeps = nuScenesConfig.GetSceneSweeps(scene);
        std::cout << "[" << sceneSweeps.size() << "] "
                  << scene["token"].asString() << ": sweeps.size = " << sweeps.size() << std::endl;
        sceneSweeps.push_back(sweeps);
        // break;
    }


    pcl::visualization::PCLVisualizer::Ptr viewer(new pcl::visualization::PCLVisualizer("NuScenes PCL Viz"));
    viewer->initCameraParameters();
    viewer->addCoordinateSystem(1.0, "coord_zero");
    viewer->setCameraPosition(
        0, 0, 30,
        0, 0, 0,
        0, 1, 0); // 0, 1, 0

    int scene_id = 0;
    int scene_size = sceneSweeps.size();
    int cloud_id = 0;

    // NOTE: Trick on how to pass lambda with capture as a callback
    // http://bannalia.blogspot.com/2016/07/passing-capturing-c-lambda-functions-as.html
    auto keyboardHandler = [&scene_id, &cloud_id, &sceneSweeps](
            const pcl::visualization::KeyboardEvent& event) {
        if (event.getKeySym() == "n" && event.keyDown()) {
            scene_id = (scene_id + 1) % sceneSweeps.size();
            cloud_id = 0;
            std::cout << "Changed sceene to: " << scene_id << std::endl;
        }
    };
    auto thunk = [](const pcl::visualization::KeyboardEvent& event, void* callback) {
        (*static_cast<decltype(keyboardHandler)*>(callback))(event);
    };

    viewer->registerKeyboardCallback(thunk, &keyboardHandler);

    std::string cloud_name = "cloud";

    pcl::PointCloud<pcl::PointXYZI>::Ptr cloud_ptr(new pcl::PointCloud<pcl::PointXYZI>());
    ReadPCDBin(sceneSweeps[scene_id][cloud_id].filename, cloud_ptr);

    pcl::visualization::PointCloudColorHandler<pcl::PointXYZI>::Ptr color_handler;
    // color_handler = boost::make_shared<pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZI> >(cloud_ptr, 255, 255, 255);
    color_handler = boost::make_shared<pcl::visualization::PointCloudColorHandlerGenericField<pcl::PointXYZI> >(cloud_ptr, "intensity");

    viewer->addPointCloud<pcl::PointXYZI>(cloud_ptr, *color_handler, cloud_name, 0);
    viewer->setPointCloudRenderingProperties(pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 1, cloud_name);

    // Preloading all clouds for quick view
    std::map<std::string, pcl::PointCloud<pcl::PointXYZI>::Ptr> clouds;
    // std::cout << "Pre-loading all clouds:\n";
    // for (auto& sweeps: sceneSweeps) {
    //     for (auto& sweep : sweeps) {
    //         pcl::PointCloud<pcl::PointXYZI>::Ptr cptr(new pcl::PointCloud<pcl::PointXYZI>());
    //         ReadPCDBin(sweep.filename, cptr);
    //         clouds[sweep.filename] = cptr;
    //     }
    //     break;
    // }

    std::cout << "Viz loop started.\n";

    while (!viewer->wasStopped()) {
        viewer->spinOnce(15);

        cloud_ptr->clear();

        cloud_id = (cloud_id + 1) % sceneSweeps[scene_id].size();
        if (cloud_id == 0 && scene_size > 1) {
            scene_id = (scene_id + 1) % scene_size;
        }

        // Get cloud points from cache or load from file
        auto fit = clouds.find(sceneSweeps[scene_id][cloud_id].filename);
        if (fit != clouds.end()) {
            cloud_ptr = fit->second;
        } else {
            cloud_ptr = boost::make_shared<pcl::PointCloud<pcl::PointXYZI>>();
            ReadPCDBin(sceneSweeps[scene_id][cloud_id].filename, cloud_ptr);
        }
            
        // color_handler = boost::make_shared<pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZI> >(next_cloud_ptr, 255, 255, 255);
        color_handler = boost::make_shared<pcl::visualization::PointCloudColorHandlerGenericField<pcl::PointXYZI> >(cloud_ptr, "intensity");

        viewer->updatePointCloud<pcl::PointXYZI>(cloud_ptr, *color_handler, cloud_name);

    }    

    return EXIT_SUCCESS;
}