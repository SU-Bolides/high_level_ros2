import unittest

import pytest

from bolide_obstacle_avoidance.obstacle_avoidance_node import ObstacleAvoidance


class TestObstacleAvoidance(unittest.TestCase):

    @pytest.mark.rostest
    def test_one_plus_one(self):
        assert 1 + 1 == 2