"""Round 4: who painted the door? Seeds (254) vs inflation (253), and do the
seeds coincide with ToF obstacle points or contact marks."""
import math, struct
from mcap_ros2.reader import read_ros2_messages

BAG = "/private/tmp/claude-501/-Users-jsperson-source-sphero-rvr-ros/88c047c7-739a-43f8-a371-25aeb91bb240/scratchpad/basin.mcap"
WINDOW = (1787262267, 1787262561)
T_OPEN, T_CLOSED = WINDOW[0] + 221, WINDOW[0] + 250

grid_open = grid_closed = None
tof_pts, mark_pts, promotes = [], [], []
for m in read_ros2_messages(BAG, topics=["/global_costmap/costmap_raw",
                                         "/tof/obstacles", "/contact_marks",
                                         "/contact_marks/promote"]):
    t = m.log_time_ns / 1e9
    msg = m.ros_msg
    topic = m.channel.topic
    if topic == "/global_costmap/costmap_raw":
        if t <= T_OPEN or grid_open is None: grid_open = (t, msg)
        if t <= T_CLOSED or grid_closed is None: grid_closed = (t, msg)
    elif topic == "/contact_marks/promote":
        promotes.append(t)
    elif T_OPEN <= t <= T_CLOSED + 10:
        # PointCloud2 xyz float32 assumed at offsets 0,4,8
        pts = []
        step = msg.point_step
        data = bytes(msg.data)
        for i in range(msg.width * msg.height):
            x, y = struct.unpack_from("<ff", data, i * step)[:2]
            pts.append((x, y))
        (tof_pts if topic == "/tof/obstacles" else mark_pts).append((t, pts, msg.header.frame_id))

def unpack(pair):
    t, msg = pair
    return {"res": msg.metadata.resolution, "w": msg.metadata.size_x,
            "h": msg.metadata.size_y, "ox": msg.metadata.origin.position.x,
            "oy": msg.metadata.origin.position.y, "data": bytes(msg.data), "t": t}

go, gc = unpack(grid_open), unpack(grid_closed)
new254, new253 = set(), set()
for i, (a, b) in enumerate(zip(go["data"], gc["data"])):
    if b >= 253 > a:
        (new254 if b == 254 else new253).add((i % gc["w"], i // gc["w"]))
print(f"door: {len(new254)} new LETHAL(254) seeds, {len(new253)} new INSCRIBED(253) inflation")
print(f"promote events at t-offsets: {[round(p - WINDOW[0],1) for p in promotes]}")

def cell(x, y):
    return (int((x - gc["ox"]) / gc["res"]), int((y - gc["oy"]) / gc["res"]))

frames = {f for _, _, f in tof_pts} | {f for _, _, f in mark_pts}
print("cloud frames seen:", frames)
tof_hits = sum(1 for _, pts, _ in tof_pts for p in pts if cell(*p) in new254)
tof_total = sum(len(pts) for _, pts, _ in tof_pts)
mark_hits = sum(1 for _, pts, _ in mark_pts for p in pts if cell(*p) in new254)
mark_total = sum(len(pts) for _, pts, _ in mark_pts)
print(f"tof obstacle points landing on new-lethal seeds: {tof_hits}/{tof_total}")
print(f"contact-mark points landing on new-lethal seeds: {mark_hits}/{mark_total}")
seed_cells_hit_tof = len({cell(*p) for _, pts, _ in tof_pts for p in pts} & new254)
seed_cells_hit_marks = len({cell(*p) for _, pts, _ in mark_pts for p in pts} & new254)
print(f"seed cells covered: by tof {seed_cells_hit_tof}/{len(new254)}, "
      f"by marks {seed_cells_hit_marks}/{len(new254)}")
