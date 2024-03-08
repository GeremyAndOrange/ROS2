Project Structure:
├── src
    ├── calculator
    │   ├── CMakeLists.txt
    │   ├── include
    │   │   └── calculator
    │   ├── package.xml
    │   └── src
    ├── manager
    │   ├── CMakeLists.txt
    │   ├── include
    │   │   └── manager
    │   │       └── manager.hpp
    │   ├── package.xml
    │   └── src
    │       ├── manager.cpp
    │       └── nodeStart.cpp
    ├── robot_control
    │   ├── CMakeLists.txt
    │   ├── include
    │   │   └── robot_control
    │   ├── package.xml
    │   └── src
    └── robot_description
        ├── launch
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
        │   └── robot.urdf
        └── world
└── README.md