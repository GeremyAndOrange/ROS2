Project Structure:
├── src
    ├── calculator
    │   ├── CMakeLists.txt
    │   ├── include
    │   │   └── calculator
    │   │       ├── common.hpp
    │   │       ├── function.hpp
    │   │       └── marco.hpp
    │   ├── package.xml
    │   └── src
    │       └── function.cpp
    ├── interfaces
    │   ├── CMakeLists.txt
    │   ├── package.xml
    │   └── srv
    │       ├── GetTask.srv
    │       └── NewRobot.srv
    ├── manager
    │   ├── CMakeLists.txt
    │   ├── include
    │   │   └── manager
    │   │       ├── manager.hpp
    │   │       └── manager_interfaces.hpp
    │   ├── package.xml
    │   └── src
    │       ├── manager.cpp
    │       └── manager_node_start.cpp
    ├── README.md
    ├── robot_control
    │   ├── CMakeLists.txt
    │   ├── include
    │   │   └── robot_control
    │   │       ├── control.hpp
    │   │       └── robot_interfaces.hpp
    │   ├── package.xml
    │   └── src
    │       ├── control.cpp
    │       └── robot_node_start.cpp
    └── robot_description
        ├── launch
        │   └── robot.launch.py
        ├── package.xml
        ├── resource
        │   └── robot_description
        ├── robot_description
        │   └── __init__.py
        ├── setup.cfg
        ├── setup.py
        ├── test
        │   ├── test_copyright.py
        │   ├── test_flake8.py
        │   └── test_pep257.py
        ├── urdf
        │   ├── robot_0.urdf
        │   ├── robot_1.urdf
        │   └── robot.urdf
        └── world
            ├── garage.world
            ├── hard.world
            ├── house.world
            └── shopping_mall.world