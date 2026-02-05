# Contributing to ROS Laser Bot

Thank you for your interest in contributing to ROS Laser Bot! This document provides guidelines and instructions for contributing to this project.

## Code of Conduct

- Be respectful and considerate
- Welcome newcomers and help them learn
- Focus on constructive feedback
- Respect different viewpoints and experiences

## How to Contribute

### Reporting Bugs

If you find a bug, please open an issue with:
- Clear description of the problem
- Steps to reproduce
- Expected vs actual behavior
- System information (OS, ROS version)
- Relevant error messages/logs

### Suggesting Enhancements

For feature requests:
- Describe the enhancement clearly
- Explain why it would be useful
- Provide examples if possible

### Pull Requests

1. **Fork the [repository](https://github.com/Shahrukh19S/ros-autonomous-laser-bot)**
2. **Create a feature branch**
   ```bash
   git checkout -b feature/your-feature-name
   ```
3. **Make your changes**
   - Follow existing code style
   - Add comments for complex logic
   - Update documentation if needed
4. **Test your changes**
   - Ensure the project builds successfully
   - Test with different controllers
   - Verify no regressions
5. **Commit your changes**
   ```bash
   git commit -m "Add: Description of your changes"
   ```
6. **Push to your fork**
   ```bash
   git push origin feature/your-feature-name
   ```
7. **Open a Pull Request**
   - Provide a clear description
   - Reference related issues
   - Include screenshots/videos if applicable

## Coding Standards

### Python Style

- Follow PEP 8 style guide
- Use meaningful variable names
- Add docstrings to classes and functions
- Keep functions focused and small

### ROS Conventions

- Use ROS naming conventions for topics and nodes
- Follow ROS message naming standards
- Document launch file parameters
- Include proper error handling

### Code Comments

- Comment complex algorithms
- Explain non-obvious logic
- Keep comments up-to-date with code
- Use clear, concise language

## Project Structure

When adding new features:
- Controllers go in `catkin_ws/src/ros2/scripts/`
- World files go in `catkin_ws/src/a2_world/world/`
- Launch files go in `catkin_ws/src/a2_world/launch/`
- Documentation goes in `docs/`

## Testing

Before submitting:
- Test on clean ROS Kinetic installation
- Verify all controllers work
- Check for memory leaks
- Ensure proper error handling

## Documentation

- Update README.md for major changes
- Add examples for new features
- Update BUILD_INSTRUCTIONS.md if setup changes
- Document new controller parameters

## Questions?

Feel free to open an issue for questions or discussions. We're happy to help!

---

Thank you for contributing to ROS Laser Bot! 🚀
