"""Test suite for multi_robots_explore package."""
import pytest


def test_package_import():
    """Test that the package can be imported."""
    try:
        import multi_robots_explore
        assert multi_robots_explore is not None
    except ImportError:
        pytest.fail("Failed to import multi_robots_explore package")


def test_nodes_exist():
    """Test that all node modules exist."""
    modules = [
        'frontier_detector',
        'goal_relay',
        'map_merger',
        'map_tf_broadcaster',
        'path_visualizer',
        'robot_position_publisher',
        'target_assigner',
        'voronoi_centroid',
    ]
    
    for module_name in modules:
        try:
            __import__(f'multi_robots_explore.{module_name}')
        except ImportError as e:
            pytest.fail(f"Failed to import module {module_name}: {e}")
