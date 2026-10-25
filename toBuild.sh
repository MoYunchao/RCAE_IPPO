rm -rf build log install
find . -type f \( -name "*.sh" -o -name "*.py" \) -exec chmod +x {} +
colcon build --symlink-install