import json, time, zmq
from scipy.spatial.transform import Rotation as R

ctx = zmq.Context()
s = ctx.socket(zmq.REQ)
s.connect("tcp://127.0.0.1:5555")
s.setsockopt(zmq.RCVTIMEO, 1000)


def fmt(d):
    if not d:
        return "n/a"
    p = d["position"]
    o = d["orientation"]
    e = R.from_quat([o["x"], o["y"], o["z"], o["w"]]).as_euler("xyz", degrees=True)
    px, py, pz = p["x"], p["y"], p["z"]
    return "pos({:+.2f},{:+.2f},{:+.2f}) rpy({:+.0f},{:+.0f},{:+.0f})".format(
        px, py, pz, e[0], e[1], e[2]
    )


end = time.time() + 20
while time.time() < end:
    try:
        s.send_string("get_vive_data")
        m = json.loads(s.recv_string())
    except Exception as ex:
        print("poll err", ex)
        time.sleep(0.5)
        continue
    print("L " + fmt(m.get("left_wrist")) + " | R " + fmt(m.get("right_wrist")), flush=True)
    time.sleep(0.5)
