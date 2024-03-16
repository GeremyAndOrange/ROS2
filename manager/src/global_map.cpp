#include "manager/global_map.hpp"

GlobalMap::GlobalMap(std::string name) : Node(name)
{
    // property

    // service

    // topic
    this->PublisherMap = this->create_publisher<nav_msgs::msg::OccupancyGrid>("/map",10);
    this->SubscriptionSubmap = this->create_subscription<nav_msgs::msg::OccupancyGrid>("/submap",10,std::bind(&GlobalMap::SubMap,this,_1));
    this->SubscriptionTF = this->create_subscription<tf2_msgs::msg::TFMessage>("/tf",10,std::bind(&GlobalMap::SubTF,this,_1));

    // timer
    this->MapTimer = this->create_wall_timer(std::chrono::seconds(1),std::bind(&GlobalMap::PubMap,this));
}

void GlobalMap::Initial()
{
    for (size_t i = 0; i < ROBOT_NUMBER; i++) {
        this->tfGroup[i].tf.x = 0.0;
        this->tfGroup[i].tf.y = 0.0;
    }
    
}

void GlobalMap::SubTF(const tf2_msgs::msg::TFMessage::SharedPtr info)
{
    std::regex str_match ("robot_(\\d+)_submap");
    for (size_t i = 0; i < info->transforms.size(); i++) {
        if (info->transforms[i].header.frame_id != "map" && !std::regex_match(info->transforms[i].child_frame_id, str_match)) {
            continue;
        } else {
            std::string sub_frame = info->transforms[i].child_frame_id;
            std::smatch match;
            std::regex_search(sub_frame, match, str_match);
            std::string id = match.str(1);
            this->tfGroup[std::stoi(id)].tf.x = info->transforms[i].transform.translation.x;
            this->tfGroup[std::stoi(id)].tf.y = info->transforms[i].transform.translation.y;
        }
    }
}

void GlobalMap::SubMap(const nav_msgs::msg::OccupancyGrid::SharedPtr info)
{
    // 坐标变换
    std::string sub_frame = info->header.frame_id;
    size_t start = sub_frame.find("robot_") + std::string("robot_").size();
    size_t end = sub_frame.find("_submap");
    std::string id = sub_frame.substr(start, end - start);
    geometry_msgs::msg::Pose origin;
    origin.position.x = info->info.origin.position.x + this->tfGroup[std::stoi(id)].tf.x;
    origin.position.y = info->info.origin.position.y + this->tfGroup[std::stoi(id)].tf.y;
    // 更新地图
    if (this->map.data.empty())
    {
        this->map.data = info->data;
        this->map.height = info->info.height;
        this->map.width = info->info.width;
        this->map.origin.x = origin.position.x;
        this->map.origin.y = origin.position.y;
    }
    else
    {
        // 折叠地图
        std::vector<std::vector<int>> twoDimMap(this->map.height, std::vector<int>(this->map.width));
        // 填充地图
        for (int i = 0; i < this->map.height; ++i) {
            for (int j = 0; j < this->map.width; ++j) {
                twoDimMap[i][j] = this->map.data[i * this->map.width + j];
            }
        }
        int left = std::ceil((this->map.origin.x - origin.position.x)/MAP_RESOLUTION);
        int right = std::ceil(this->map.origin.x/MAP_RESOLUTION + this->map.width - origin.position.x/MAP_RESOLUTION - info->info.width);
        int down = std::ceil((this->map.origin.y - origin.position.y)/MAP_RESOLUTION);
        int up = std::ceil(this->map.origin.y/MAP_RESOLUTION + this->map.height - origin.position.y/MAP_RESOLUTION - info->info.height);
        int update_coordinate_x = -down;
        int update_coordinate_y = -left;
        if (left > 0) {
            this->map.origin.x = origin.position.x;
            this->map.width = this->map.width + left;
            update_coordinate_y = 0;
            // 扩展地图
            for (auto& row : twoDimMap) {
                row.insert(row.begin(), left, -1);
            }
        }
        if (right < 0) {
            this->map.width = this->map.width - right;
            // 扩展地图
            for (auto& row : twoDimMap) {
                row.insert(row.end(), -right, -1);
            }
        }
        std::vector<int> new_row(this->map.width, -1);
        if (down > 0) {
            this->map.origin.y = origin.position.y;
            this->map.height = this->map.height + down;
            update_coordinate_x = 0;
            // 扩展地图
            for (int i = 0; i < down; ++i) {
                twoDimMap.insert(twoDimMap.begin(), new_row);
            }
        }
        if (up < 0) {
            this->map.height = info->info.height - up;
            // 扩展地图
            for (int i = 0; i < -up; ++i) {
                twoDimMap.insert(twoDimMap.end(), new_row);
            }
        }
        // 更新地图
        for (size_t i = 0; i < info->info.height; i++) {
            for (size_t j = 0; j < info->info.width; j++) {
                twoDimMap[update_coordinate_x + i][update_coordinate_y + j] = info->data[i * info->info.width + j];
            }
        }
        // 展开地图
        std::vector<int8_t> mapData;
        for (const auto& row : twoDimMap) {
            for (const auto& element : row) {
                mapData.push_back(static_cast<int8_t>(element));
            }
        }
        this->map.data = mapData;
    }
}

void GlobalMap::PubMap()
{
    nav_msgs::msg::OccupancyGrid info;
    info.header.frame_id = "map";
    info.header.stamp = this->get_clock()->now();
    info.info.map_load_time = this->get_clock()->now();
    info.info.height = this->map.height;
    info.info.width = this->map.width;
    info.info.resolution = MAP_RESOLUTION;
    info.info.origin.position.x = this->map.origin.x;
    info.info.origin.position.y = this->map.origin.y;
    info.data = this->map.data;
    this->PublisherMap->publish(info);
}