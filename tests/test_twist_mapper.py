import math

import pytest

from sphero_rvr_driver.twist_mapper import TwistLike, map_twist_to_velocity


def test_twist_mapper_clamps_linear_and_angular_velocity():
    twist = TwistLike(linear_x=4.0, angular_z=-9.0)

    velocity = map_twist_to_velocity(twist, max_linear_mps=0.5, max_angular_rad_s=1.2)

    assert velocity.linear_mps == 0.5
    assert velocity.angular_rad_s == -1.2


def test_twist_mapper_preserves_values_inside_limits():
    twist = TwistLike(linear_x=-0.25, angular_z=0.75)

    velocity = map_twist_to_velocity(twist, max_linear_mps=0.5, max_angular_rad_s=1.2)

    assert velocity.linear_mps == -0.25
    assert velocity.angular_rad_s == 0.75


@pytest.mark.parametrize("linear_x", (math.nan, math.inf, -math.inf))
def test_twist_mapper_rejects_non_finite_linear_velocity(linear_x):
    with pytest.raises(ValueError, match="non-finite"):
        map_twist_to_velocity(TwistLike(linear_x=linear_x, angular_z=0.0), max_linear_mps=0.5, max_angular_rad_s=1.2)


@pytest.mark.parametrize("angular_z", (math.nan, math.inf, -math.inf))
def test_twist_mapper_rejects_non_finite_angular_velocity(angular_z):
    with pytest.raises(ValueError, match="non-finite"):
        map_twist_to_velocity(TwistLike(linear_x=0.0, angular_z=angular_z), max_linear_mps=0.5, max_angular_rad_s=1.2)
