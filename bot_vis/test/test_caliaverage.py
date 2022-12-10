from bot_vis.calibration import average_points


def test_caliaverage():
    # test file for average function
    x,y,z,qx,qy,qz,qw = average_points([1,3],[2,4],[3,5],[4,6],[5,7],[6,8],[7,9])
    assert x == 2.0
    assert y == 3.0
    assert z == 4.0