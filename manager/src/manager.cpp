#include "manager/manager.hpp"

// 构造函数,有一个参数为节点名称
ManagerNode::ManagerNode(std::string name) : Node(name)
{
    RCLCPP_INFO(this->get_logger(), "This is manager node.");
    this->Initial();
}

ManagerNode::~ManagerNode()
{
    
}

void ManagerNode::Initial()
{
    // ptr
    TfBuffer = std::make_unique<tf2_ros::Buffer>(this->get_clock());
    TfListener = std::make_shared<tf2_ros::TransformListener>(*TfBuffer);

    // service
    this->SendTaskService = this->create_service<interfaces::srv::GetTask>("/get_task",std::bind(&ManagerNode::SendTask,this,_1,_2));

    // topic
    this->SubscriptionGLobalMap = this->create_subscription<nav_msgs::msg::OccupancyGrid>("/globalmap",10,std::bind(&ManagerNode::SubMap,this,_1));
    this->PublisherExpansionedMap = this->create_publisher<nav_msgs::msg::OccupancyGrid>("/expansion_map",10);
    this->PublisherBoundaryPoints = this->create_publisher<visualization_msgs::msg::Marker>("/boundary", 10);

    // timer
    
}

void ManagerNode::SendTask(const interfaces::srv::GetTask::Request::SharedPtr request, const interfaces::srv::GetTask::Response::SharedPtr response)
{
    // save robot position
    RobotInfo robot;
    robot.id = request->id;
    robot.tf.x = request->tf_x;
    robot.tf.y = request->tf_y;
    robot.coor.x = request->coor_x;
    robot.coor.y = request->coor_y;
    RobotGroup[robot.id] = robot;

    std::lock_guard<std::mutex> lock(this->MapMutex);
    // calculate expansion map
    ExpansionMap();

    // odom -> world
    Coordinate PointInMap;
    geometry_msgs::msg::TransformStamped OdomToWorld;
    try {
        OdomToWorld = TfBuffer->lookupTransform("world", "robot_" + robot.id + "_odom", tf2::TimePointZero);
        
        // coordinate in world
        Coordinate PositionInWorld;
        PositionInWorld.x = robot.coor.x + OdomToWorld.transform.translation.x;
        PositionInWorld.y = robot.coor.y + OdomToWorld.transform.translation.y;

        // coordinate in map
        Coordinate PositionInMap;
        PositionInMap.x = PositionInWorld.x - this->ExpansionedMap.info.origin.position.x;
        PositionInMap.y = PositionInWorld.y - this->ExpansionedMap.info.origin.position.y;
        
        // coordinate index relative origin
        PointInMap.x = std::round(PositionInMap.x / this->ExpansionedMap.info.resolution);
        PointInMap.y = std::round(PositionInMap.y / this->ExpansionedMap.info.resolution);
    }
    catch(const tf2::TransformException &error) {
        response->path = {};
        response->is_path = false;
        RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "TF Exception: %s", error.what());
        return;
    }

    // calculate task points
    bool bRets = false;
    std::vector<Coordinate> BoundaryPoints;                             // coordinate in map
    bRets = ScanBoundary(PointInMap, BoundaryPoints);

    visualization_msgs::msg::Marker BoundaryMarker;
    BoundaryMarker.header.frame_id = "world";
    BoundaryMarker.header.stamp = this->get_clock()->now();
    BoundaryMarker.ns = "boundary";
    BoundaryMarker.id = 0;
    BoundaryMarker.type = visualization_msgs::msg::Marker::POINTS;
    BoundaryMarker.action = visualization_msgs::msg::Marker::ADD;

    BoundaryMarker.scale.x = 0.05;
    BoundaryMarker.scale.y = 0.05;
    BoundaryMarker.scale.z = 0;

    BoundaryMarker.color.a = 1.0;
    BoundaryMarker.color.r = 0.0;
    BoundaryMarker.color.g = 1.0;
    BoundaryMarker.color.b = 0.0;

    for (const auto& point : BoundaryPoints) {
        geometry_msgs::msg::Point AddedPoint;
        AddedPoint.x = point.x * this->ExpansionedMap.info.resolution + this->ExpansionedMap.info.origin.position.x;
        AddedPoint.y = point.y * this->ExpansionedMap.info.resolution + this->ExpansionedMap.info.origin.position.y;
        AddedPoint.z = 0;
        BoundaryMarker.points.push_back(AddedPoint);
    }
    this->PublisherBoundaryPoints->publish(BoundaryMarker);

    if (bRets) {
        // calculate nearest task point
        Coordinate TargetPoint;
        std::vector<Coordinate> TaskPoints;
        bRets = AggregateTask(BoundaryPoints, TaskPoints);

        visualization_msgs::msg::Marker TaskMarker;
        TaskMarker.header.frame_id = "world";
        TaskMarker.header.stamp = this->get_clock()->now();
        TaskMarker.ns = "task";
        TaskMarker.id = 1;
        TaskMarker.type = visualization_msgs::msg::Marker::POINTS;
        TaskMarker.action = visualization_msgs::msg::Marker::ADD;

        TaskMarker.scale.x = 0.15;
        TaskMarker.scale.y = 0.15;
        TaskMarker.scale.z = 0;

        TaskMarker.color.a = 1.0;
        TaskMarker.color.r = 0.0;
        TaskMarker.color.g = 1.0;
        TaskMarker.color.b = 1.0;

        for (const auto& point : TaskPoints) {
            geometry_msgs::msg::Point AddedPoint;
            AddedPoint.x = point.x * this->ExpansionedMap.info.resolution + this->ExpansionedMap.info.origin.position.x;
            AddedPoint.y = point.y * this->ExpansionedMap.info.resolution + this->ExpansionedMap.info.origin.position.y;
            AddedPoint.z = 0;
            TaskMarker.points.push_back(AddedPoint);
        }
        this->PublisherBoundaryPoints->publish(TaskMarker);

        if (bRets) {
            // calculate path planning
            std::vector<Coordinate> TaskPathPlanning;
            while (!TaskPoints.empty()) {
                double MinDist = std::numeric_limits<double>::max();
                for (const auto& task : TaskPoints) {
                    double dist = CalDistance(robot.coor, task);
                    if (dist < MinDist) {
                        TargetPoint = task;                                 // coordinate in map
                        MinDist = dist;
                    }
                }

                bRets = DijsktraAlgorithm(PointInMap, TargetPoint, TaskPathPlanning);
                if (bRets) {
                    break;
                } else {
                    auto it = std::find(TaskPoints.begin(), TaskPoints.end(), TargetPoint);
                    if (it != TaskPoints.end()) {
                        TaskPoints.erase(it);
                    }
                }
            }

            if (bRets) {
                visualization_msgs::msg::Marker TargetMarker;
                TargetMarker.header.frame_id = "world";
                TargetMarker.header.stamp = this->get_clock()->now();
                TargetMarker.ns = "target";
                TargetMarker.id = 2;
                TargetMarker.type = visualization_msgs::msg::Marker::POINTS;
                TargetMarker.action = visualization_msgs::msg::Marker::ADD;

                TargetMarker.scale.x = 0.15;
                TargetMarker.scale.y = 0.15;
                TargetMarker.scale.z = 0;

                TargetMarker.color.a = 1.0;
                TargetMarker.color.r = 1.0;
                TargetMarker.color.g = 1.0;
                TargetMarker.color.b = 1.0;

                geometry_msgs::msg::Point AddedTargetPoint;
                AddedTargetPoint.x = TargetPoint.x * this->ExpansionedMap.info.resolution + this->ExpansionedMap.info.origin.position.x;
                AddedTargetPoint.y = TargetPoint.y * this->ExpansionedMap.info.resolution + this->ExpansionedMap.info.origin.position.y;
                AddedTargetPoint.z = 0;
                TargetMarker.points.push_back(AddedTargetPoint);
                this->PublisherBoundaryPoints->publish(TargetMarker);

                // skip path points
                std::vector<int> RemoveIndex;
                for (size_t i = 1; i <TaskPathPlanning.size()-1; i++) {
                    Coordinate LastPoint = TaskPathPlanning[i-1];
                    Coordinate NextPoint = TaskPathPlanning[i+1];
                    Coordinate CurrentPoint = TaskPathPlanning[i];
                    if ((CurrentPoint.x - LastPoint.x) == (NextPoint.x - CurrentPoint.x) && (CurrentPoint.y - LastPoint.y) == (NextPoint.y - CurrentPoint.y)) {
                        RemoveIndex.push_back(i);
                    }
                }
                for (auto it = RemoveIndex.rbegin(); it != RemoveIndex.rend(); ++it) {
                    TaskPathPlanning.erase(TaskPathPlanning.begin() + *it);
                }

                RemoveIndex.clear();
                for (size_t i = 0; i <TaskPathPlanning.size()-1; i++) {
                    Coordinate NextPoint = TaskPathPlanning[i+1];
                    Coordinate CurrentPoint = TaskPathPlanning[i];
                    if (CalDistance(CurrentPoint, NextPoint) <= 3) {
                        RemoveIndex.push_back(i);
                    }
                }
                for (auto it = RemoveIndex.rbegin(); it != RemoveIndex.rend(); ++it) {
                    TaskPathPlanning.erase(TaskPathPlanning.begin() + *it);
                }

                // send task path
                std::vector<double> target_path;
                for (const auto& point : TaskPathPlanning) {
                    Coordinate TargetRealPotion;
                    TargetRealPotion.x = point.x * this->ExpansionedMap.info.resolution + this->ExpansionedMap.info.origin.position.x;
                    TargetRealPotion.y = point.y * this->ExpansionedMap.info.resolution + this->ExpansionedMap.info.origin.position.y;
                    target_path.push_back(TargetRealPotion.x);
                    target_path.push_back(TargetRealPotion.y);
                }
                response->is_path = true;
                response->path = target_path;
            }
            else {
                RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "NO PATH.");
                response->is_path = false;
                response->path = {};
            }
        }
        else {
            RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "NO CALCULATE TASK POINTS.");
            response->is_path = false;
            response->path = {};
        }
    }   
    else {
        RCLCPP_ERROR_THROTTLE(this->get_logger(), *this->get_clock(), 1000, "NO SCAN BOUNDARY.");
        response->is_path = false;
        response->path = {};
    }
}

void ManagerNode::SubMap(const nav_msgs::msg::OccupancyGrid::SharedPtr info)
{
    std::lock_guard<std::mutex> lock(this->MapMutex);
    this->StoredMap = *info;
}

void ManagerNode::ExpansionMap()
{
    this->ExpansionedMap = this->StoredMap;

    uint32_t width = this->StoredMap.info.width;
    uint32_t height = this->StoredMap.info.height;
    double_t resolution = this->StoredMap.info.resolution;

    int ExpansionRadius = int(std::round(EXPANSION_RADIUS / resolution));
    for (size_t y = 0; y < height; y++) {
        for (size_t x = 0; x < width; x++) {
            if (this->StoredMap.data[y * width + x] > 65) {
                for (int dy = -2 * ExpansionRadius; dy <= 2 * ExpansionRadius; ++dy) {
                    for (int dx = -2 * ExpansionRadius; dx <= 2 * ExpansionRadius; ++dx) {
                        if (this->StoredMap.data[(y+dy) * width + (x+dx)] <= 65 && this->StoredMap.data[(y+dy) * width + (x+dx)] != -1) {
                            if ((x+dx) > 0 && (x+dx) < width && (y+dy) > 0 && (y+dy) < height) {
                                this->ExpansionedMap.data[(y+dy) * width + (x+dx)] = 75;
                            }
                        }
                    }
                }
            }
        }
    }
    
    this->PublisherExpansionedMap->publish(this->ExpansionedMap);
}

bool ManagerNode::ScanBoundary(Coordinate PointInMap, std::vector<Coordinate>& BoundaryPoints)
{
    int IterationCoff = 1;
    while (IterationCoff <= 5) {
        // find boundary
        std::vector<Coordinate> RRTree;
        RRTree.push_back(PointInMap);    
        srand(static_cast<unsigned int>(time(nullptr)));

        int StepSize = int(std::round(RRT_STEP_SIZE / this->ExpansionedMap.info.resolution));
        int MaxIteration = IterationCoff * int(std::round((this->ExpansionedMap.info.width / StepSize))) * int(std::round((this->ExpansionedMap.info.height / StepSize)));
        for (int i = 0; i < MaxIteration; i++) {
            Coordinate RandomPoint = GenerateRandomNode();
            Coordinate NearestPoint = RRTree[0];
            double NearestDist = CalDistance(RandomPoint, NearestPoint);
            for (const auto& point : RRTree) {
                double dist = CalDistance(RandomPoint, point);
                if (dist < NearestDist) {
                    NearestDist = dist;
                    NearestPoint = point;
                }
            }
            Coordinate NewPoint = FindNewNode(NearestPoint, RandomPoint);

            if (IsValidPoint(NewPoint)) {
                int MapValue = this->ExpansionedMap.data[int(NearestPoint.y) * this->ExpansionedMap.info.width + (NearestPoint.x)];
                if (MapValue != -1) {
                    RRTree.push_back(NewPoint);
                    if (this->ExpansionedMap.data[int(NewPoint.y) * this->ExpansionedMap.info.width + int(NewPoint.x)] == -1  && MapValue != 75) {
                        BoundaryPoints.push_back(NewPoint); 
                    }
                }
            }
        }

        // check scan result
        if (BoundaryPoints.size() < 4 * this->RobotGroup.size()) {
            IterationCoff += 1;
        }
        else {
            return true;
        }
    }
    return false;
}

bool ManagerNode::IsValidPoint(Coordinate point)
{
    uint32_t width = this->ExpansionedMap.info.width;
    uint32_t height = this->ExpansionedMap.info.height;

    uint32_t x = uint32_t(point.x);
    uint32_t y = uint32_t(point.y);
    return  x < width && y < height && (this->ExpansionedMap.data[y * width + x] <= 65 || this->ExpansionedMap.data[y * width + x] == 75);
}

Coordinate ManagerNode::GenerateRandomNode()
{
    Coordinate RandomPoint;
    RandomPoint.x = rand() % this->ExpansionedMap.info.width;
    RandomPoint.y = rand() % this->ExpansionedMap.info.height;
    return RandomPoint;
}

Coordinate ManagerNode::FindNewNode(Coordinate LastNode, Coordinate NextNode)
{
    Coordinate NewNode;
    double_t resolution = this->ExpansionedMap.info.resolution;
    double dist = CalDistance(LastNode, NextNode) * resolution;
    if (dist < RRT_STEP_SIZE) {
        NewNode = NextNode;
    }
    else {
        NewNode.x = LastNode.x + std::round((NextNode.x - LastNode.x) * RRT_STEP_SIZE / dist);
        NewNode.y = LastNode.y + std::round((NextNode.y - LastNode.y) * RRT_STEP_SIZE / dist);
    }
    
    return NewNode;
}

bool ManagerNode::AggregateTask(const std::vector<Coordinate>& BoundaryPoints, std::vector<Coordinate>& TaskPoints)
{
    int CentroidsNum =  4 * this->RobotGroup.size();
    if (int(BoundaryPoints.size()) < CentroidsNum) {
        return false;
    }

    std::vector<Coordinate> centroids;
    std::srand(static_cast<unsigned int>(std::time(nullptr)));
    for (int i = 0; i < CentroidsNum; i++) {
        int index = std::rand() % BoundaryPoints.size();
        centroids.push_back(BoundaryPoints[index]);
    }

    int IterationCount = 0;
    bool converged = false;
    while (!converged || IterationCount < KMEANS_ITERATION) {
        IterationCount += 1;
        // allocation points
        std::vector<std::vector<Coordinate>> clusters(CentroidsNum);
        for (const auto& BoundaryPoint : BoundaryPoints) {
            int ClosestCentroidIndex = 0;
            double ClosestDistance = std::numeric_limits<double>::max();
            for (int i = 0; i < CentroidsNum; ++i) {
                double distance = CalDistance(BoundaryPoint, centroids[i]);
                if (distance < ClosestDistance) {
                    ClosestDistance = distance;
                    ClosestCentroidIndex = i;
                }
            }
            clusters[ClosestCentroidIndex].push_back(BoundaryPoint);
        }

        // update centroid
        converged = true;
        for (int i = 0; i < CentroidsNum; i++) {
            Coordinate NewCentroid;

            if (clusters[i].size() > 0) {
                for (const auto& point : clusters[i]) {
                    NewCentroid.x += point.x;
                    NewCentroid.y += point.y;
                }
                NewCentroid.x = std::round(NewCentroid.x / clusters[i].size());
                NewCentroid.y = std::round(NewCentroid.y / clusters[i].size());

                if (CalDistance(NewCentroid, centroids[i]) * this->ExpansionedMap.info.resolution > PRECISION_1) {
                    centroids[i] = NewCentroid;
                    converged = false;
                }
            }
        }
    }

    if (centroids.empty()) {
        return false;
    }
    else {
        TaskPoints = centroids;
        return true;
    }
}

bool ManagerNode::DijsktraAlgorithm(Coordinate SourcePoint, Coordinate TargetPoint, std::vector<Coordinate>& TaskPathPlanning)
{
    uint32_t width = this->ExpansionedMap.info.width;
    uint32_t height = this->ExpansionedMap.info.height;

    std::vector<std::vector<bool>> visited(width, std::vector<bool>(height, false));
    std::vector<Coordinate> directions = {{0.0,1.0},{0.0,-1.0},{-1.0,0.0},{1.0,0.0}};
    std::vector<std::vector<Coordinate>> parents(width, std::vector<Coordinate>(height));
    std::vector<std::vector<double>> distances(width, std::vector<double>(height, std::numeric_limits<double>::max()));

    std::vector<Coordinate> OpenSet;
    OpenSet.push_back(SourcePoint);
    distances[int(SourcePoint.x)][int(SourcePoint.y)] = 0;
    while (!OpenSet.empty()) {
        Coordinate current = OpenSet.front();
        for (const auto& node : OpenSet) {
            if (distances[int(node.x)][int(node.y)] < distances[int(current.x)][int(current.y)]) {
                current = node;
            }
        }

        visited[int(current.x)][int(current.y)] = true;
        OpenSet.erase(OpenSet.begin());

        if (current == TargetPoint) {
            // Reconstruct path
            while (current != SourcePoint) {
                TaskPathPlanning.push_back(current);
                current = parents[int(current.x)][int(current.y)];
            }
            TaskPathPlanning.push_back(SourcePoint);
            std::reverse(TaskPathPlanning.begin(), TaskPathPlanning.end());
            return true;
        }


        for (const auto& dir : directions) {
            Coordinate neighbor = {current.x + dir.x, current.y + dir.y};
            if (neighbor.x >= 0 && neighbor.x < width && neighbor.y >= 0 && neighbor.y < height) {
                // skip used node and obstacle
                if (!visited[int(neighbor.x)][int(neighbor.y)] && (this->ExpansionedMap.data[neighbor.y * width + neighbor.x] <= 65 || this->ExpansionedMap.data[neighbor.y * width + neighbor.x] == 75)) {
                    double newDist = distances[int(current.x)][int(current.y)] + 1 + (this->ExpansionedMap.data[neighbor.y * width + neighbor.x] == 75 ? 100 : 0);
                    if (newDist < distances[neighbor.x][neighbor.y]) {
                        distances[int(neighbor.x)][int(neighbor.y)] = newDist;
                        parents[int(neighbor.x)][int(neighbor.y)] = current;
                        OpenSet.push_back(neighbor);
                    }
                }
            }
        }
    }

    return false;
}