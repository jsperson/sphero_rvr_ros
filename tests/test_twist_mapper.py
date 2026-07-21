import math

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


def test_twist_mapper_fails_non_finite_values_closed_to_zero():
    for value in (math.nan, math.inf, -math.inf):
        velocity = map_twist_to_velocity(TwistLike(linear_x=value, angular_z=value), 0.5, 1.2)

        assert velocity.linear_mps == 0.0
        assert velocity.angular_rad_s == 0.0
