from movebot.simple_move import euler_from_quaternion
import math


def test_qtoe():
    #test file for quaternion_from_euler function
    x,y,z = euler_from_quaternion(0.1, 0.0, 0.0, 1)

    assert math.isclose(x ,0.1993373,abs_tol = 0.01)
    assert math.isclose(z ,0.0,abs_tol = 0.01)