#include "current_game_info.h"
#include "mtuav_sdk.h"

// TODO 加log打印
namespace mtuav::algorithm {

std::shared_ptr<DynamicGameInfo> DynamicGameInfo::getDynamicGameInfoPtr() {
    std::call_once(singleton_flag, [&] {
        current_game_info = std::shared_ptr<DynamicGameInfo>(new DynamicGameInfo());
    });
    return current_game_info;
}

// 写入dorne、cargo 动态信息
void DynamicGameInfo::udpate_current_info(std::vector<mtuav::DroneStatus> &input_drones,
                                          std::map<int, mtuav::CargoInfo> &input_cargoes) {
    std::lock_guard<std::mutex> lock(this->_info_mutex);
    this->_current_drone_info.clear();
    this->_current_cargo_info.clear();
    for (auto &drone : input_drones) {
        this->_current_drone_info.push_back(drone);
    }
    for (auto &[id, cargo] : input_cargoes) {
        this->_current_cargo_info.insert({id, cargo});
    }
    return;
}

// 获取最新的动态信息
std::tuple<std::vector<mtuav::DroneStatus>, std::map<int, mtuav::CargoInfo>>
DynamicGameInfo::get_current_info() {
    std::lock_guard<std::mutex> lock(this->_info_mutex);
    return {this->_current_drone_info, this->_current_cargo_info};
}

// 设置任务结束标识符
void DynamicGameInfo::set_task_stop_flag(bool f) {
    std::lock_guard<std::mutex> lock(this->_info_mutex);
    this->_task_stop_flag = f;
}

// 获取任务结束标识符
bool DynamicGameInfo::get_task_stop_flag() {
    std::lock_guard<std::mutex> lock(this->_info_mutex);
    return this->_task_stop_flag;
}


// 生成新的XML字符串（只是一个示例，你可能需要根据自己的需要进行修改）
std::string GenerateMapNewXML(int width, int height, const std::vector<std::vector<int>>& grid) {
    std::ostringstream xmlStream;
    xmlStream << "<?xml version=\"1.0\" ?>\n";
    xmlStream << "<root>\n";
    xmlStream << "  <map type=\"grid\">\n";
    xmlStream << "    <grid width=\"" << width << "\" height=\"" << height << "\">\n";

    for (const auto& row : grid) {
        xmlStream << "      <row>";
        for (const auto& cell : row) {
            xmlStream << cell << " ";
        }
        xmlStream << "</row>\n";
    }

    xmlStream << "    </grid>\n";
    xmlStream << "  </map>\n";
    xmlStream << "</root>\n";
    return xmlStream.str();
}

std::string GenerateTaskNewXML(const Vec3& start, const Vec3& end) {
    std::ostringstream xmlStream;
    xmlStream << "<?xml version=\"1.0\" ?>\n";
    xmlStream << "<root>\n";
    xmlStream << "  <agents number=\"1\">\n";
    xmlStream << "    <agent id=\"0\" ";
    xmlStream << "start.x=\"" << start.x << "\" start.y=\"" << start.y << "\" ";
    xmlStream << "goal.x=\"" << end.x << "\" goal.y=\"" << end.y << "\"/>\n";
    xmlStream << "  </agents>\n";
    xmlStream << "</root>\n";
    return xmlStream.str();
}

void SaveXMLToFile(const std::string& xmlContent, std::string mode, std::string drone_id) {
    std::stringstream filePathStream;
    filePathStream << "/workspace/mtuav-competition/params/"<< mode<<"-" << drone_id << ".xml";
    std::string filePath = filePathStream.str();
    std::ofstream outFile(filePath);

    if (outFile.is_open()) {
        outFile << xmlContent;
        outFile.close();
        std::cout << "XML has been successfully saved to " << filePath << std::endl;
    } else {
        std::cout << "Unable to open file for writing: " << filePath << std::endl;
    }
}

std::vector<Vec3> ReadXMLFromFile(std::string read_dir, double flying_height){
    tinyxml2::XMLDocument doc;
    std::vector<Vec3> vecs;

    if (doc.LoadFile(read_dir.c_str()) != tinyxml2::XML_SUCCESS) {
        std::cerr << "Failed to load XML file" << std::endl;
        abort();
    }

    tinyxml2::XMLElement* root = doc.FirstChildElement("root");
    if (root) {
        tinyxml2::XMLElement* log = root->FirstChildElement("log");
        if (log) {
            tinyxml2::XMLElement* agent = log->FirstChildElement("agent");
            while (agent) {
                tinyxml2::XMLElement* path = agent->FirstChildElement("path");
                while (path) {
                    tinyxml2::XMLElement* section = path->FirstChildElement("section");
                    int sectionCount = 0;
                    while (section) {
                        sectionCount++;
                        section = section->NextSiblingElement("section");
                    }
                    section = path->FirstChildElement("section");
                    int currentCount = 0;
                    while (section) {
                        currentCount++;
                        if (currentCount < sectionCount) {
                            Vec3 vec;
                            section->QueryDoubleAttribute("goal.x", &vec.x);
                            section->QueryDoubleAttribute("goal.y", &vec.y);
                            vec.z = flying_height;  // Z值是常数
                            vecs.push_back(vec);
                        }
                        section = section->NextSiblingElement("section");
                    }
                    path = path->NextSiblingElement("path");
                }
                agent = agent->NextSiblingElement("agent");
            }
        }
    }

    return vecs;
}

}  // namespace mtuav::algorithm