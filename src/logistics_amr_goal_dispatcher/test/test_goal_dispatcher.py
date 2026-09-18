from pathlib import Path

from geometry_msgs.msg import PoseStamped
import pytest

from logistics_amr_goal_dispatcher.goal_dispatcher import (
    load_robot_names,
    validate_goal_pose,
)


def test_load_robot_names(tmp_path: Path):
    config = tmp_path / 'robots.yaml'
    config.write_text(
        'robots:\n'
        '  - name: robot1\n'
        '  - name: robot2\n',
        encoding='utf-8',
    )
    assert load_robot_names(config) == ['robot1', 'robot2']


def test_duplicate_robot_names_are_rejected(tmp_path: Path):
    config = tmp_path / 'robots.yaml'
    config.write_text(
        'robots:\n'
        '  - name: robot1\n'
        '  - name: robot1\n',
        encoding='utf-8',
    )
    with pytest.raises(RuntimeError, match='unique'):
        load_robot_names(config)


def test_validate_goal_pose():
    goal = PoseStamped()
    goal.header.frame_id = 'map'
    goal.pose.orientation.w = 1.0
    assert validate_goal_pose(goal, 'map') is None


def test_wrong_frame_is_rejected():
    goal = PoseStamped()
    goal.header.frame_id = 'odom'
    goal.pose.orientation.w = 1.0
    assert 'goal frame' in validate_goal_pose(goal, 'map')


def test_zero_quaternion_is_rejected():
    goal = PoseStamped()
    goal.header.frame_id = 'map'
    goal.pose.orientation.w = 0.0
    assert 'quaternion' in validate_goal_pose(goal, 'map')
