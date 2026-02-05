# Changelog

All notable changes to this project will be documented in this file.

The format is based on [Keep a Changelog](https://keepachangelog.com/en/1.0.0/),
and this project adheres to [Semantic Versioning](https://semver.org/spec/v2.0.0.html).

## [1.0.0] - 2026-02-06

### Added
- Initial release of ROS Laser Bot project
- Three smooth controller implementations:
  - SmoothController1: Basic sequential controller
  - SmoothController2: Advanced coordinate transformation controller
  - SmoothController1TF: TF-based controller
- Stage simulation world package (a2_world)
- Complete documentation:
  - Comprehensive README.md
  - Detailed BUILD_INSTRUCTIONS.md
  - Controller documentation (docs/CONTROLLERS.md)
  - System architecture documentation (docs/ARCHITECTURE.md)
  - Quick start guide (docs/QUICK_START.md)
- Launch files for easy execution
- Contributing guidelines (CONTRIBUTING.md)
- MIT License
- .gitignore for ROS/catkin projects

### Fixed
- Renamed CMakeLists.txt.txt to CMakeLists.txt in ros2 package
- Updated package.xml files with proper metadata and dependencies
- Added proper script installation in CMakeLists.txt

### Changed
- Improved package descriptions
- Standardized project structure
- Enhanced code documentation

## [Unreleased]

### Planned
- Obstacle avoidance integration
- Path planning capabilities
- RViz visualization configuration
- Unit tests
- CI/CD pipeline
- Docker support for easy setup

---

[1.0.0]: https://github.com/Shahrukh19S/ros-autonomous-laser-bot/releases/tag/v1.0.0
