from movebot.simple_move import quaternion_from_euler
import math

def test_etoq():
    # test file for euler_from_quaternion
    q = quaternion_from_euler(1.0,0.0,0.0)

    assert math.isclose(q[0] ,0.4794255, abs_tol = 0.01)
    assert math.isclose(q[3] ,0.8775826,abs_tol = 0.01)