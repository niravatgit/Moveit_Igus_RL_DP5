import argparse
import roypy
import time
import queue
import numpy as np
import open3d as o3d
from sample_camera_info import print_camera_info
from roypy_sample_utils import CameraOpener, add_camera_opener_options, select_use_case
from roypy_platform_utils import PlatformHelper

class MyListener(roypy.IDepthDataListener):
    def __init__(self, q):
        super(MyListener, self).__init__()
        self.queue = q
        self.figSetup = False
        self.firstTime = True
        self.vis = o3d.visualization.Visualizer()
        self.vis.create_window()
        self.pointcloud_data = None
        self.path_points = None
        self.selected_point = None  # Store selected point for path planning

    def onNewData(self, data):
        pc = data.npoints()
        px = pc[:, :, 0]
        py = pc[:, :, 1]
        pz = pc[:, :, 2]
        stack1 = np.stack([px, py, pz], axis=-1)
        stack2 = stack1.reshape(-1, 3)
        
        self.queue.put(stack2)
        self.pointcloud_data = stack2

    def select_point(self, x, y):
        if self.pointcloud_data is not None:
            self.selected_point = self.pointcloud_data[y, x]

    def generate_path(self):
        if self.selected_point is not None:
            self.path_points = np.array([self.selected_point])

    def paint(self, data):
        data = data[np.all(data != 0, axis=1)]
        vec3d = o3d.utility.Vector3dVector(data)
        
        if self.firstTime:
            self.pointcloud = o3d.geometry.PointCloud(vec3d)
            self.vis.add_geometry(self.pointcloud)
            vc = self.vis.get_view_control()
            vc.set_front([0., 0., -1.])
            self.firstTime = False
        
        self.pointcloud.points = vec3d

        if self.path_points is not None:
            if not self.figSetup:
                path_color = np.array([[1, 0, 0]])  # Red color for the path
                path_points = o3d.utility.Vector3dVector(self.path_points)
                self.path = o3d.geometry.LineSet()
                self.path.points = path_points
                self.path.lines = o3d.utility.Vector2iVector(np.array([[0, 1]]))
                self.path.colors = o3d.utility.Vector3dVector(path_color)
                self.vis.add_geometry(self.path)
                self.figSetup = True

        result = self.vis.update_geometry(self.pointcloud)
        if self.figSetup:
            self.vis.update_geometry(self.path)
        self.vis.poll_events()
        self.vis.update_renderer()

def process_event_queue(q, painter, minutes_to_display):
    t_end = time.time() + minutes_to_display * 60
    window_closed = False

    while time.time() < t_end and not window_closed:
        try:
            if len(q.queue) == 0:
                item = q.get(True, 1)
            else:
                for i in range(0, len(q.queue)):
                    item = q.get(True, 1)
        except queue.Empty:
            break
        else:
            painter.paint(item)

        window_closed = not painter.vis.poll_events()
        painter.vis.update_renderer()

    painter.vis.destroy_window()

def main():
    platformhelper = PlatformHelper()
    parser = argparse.ArgumentParser(usage=__doc__)
    add_camera_opener_options(parser)
    parser.add_argument("--minutes", type=int, default=60, help="duration to capture data in minutes")
    options = parser.parse_args()
    opener = CameraOpener(options)
    cam = opener.open_camera()

    print_camera_info(cam)
    print("isConnected", cam.isConnected())
    print("getFrameRate", cam.getFrameRate())

    curUseCase = select_use_case(cam)

    try:
        replay = cam.asReplay()
        print("Using a recording")
        print("Framecount : ", replay.frameCount())
        print("File version : ", replay.getFileVersion())
    except SystemError:
        print("Using a live camera")

    q = queue.Queue()
    l = MyListener(q)
    cam.registerDataListener(l)

    print("Setting use case : " + curUseCase)
    cam.setUseCase(curUseCase)

    cam.startCapture()
    process_event_queue(q, l, options.minutes)
    cam.stopCapture()

    l.generate_path()
    waypoints = l.path_points
    print("Waypoints:", waypoints)

    if waypoints is not None and len(waypoints) >= 2:
        print("Path Planning Trajectory: Start Point -> End Point")
        print("Start Point:", waypoints[0])
        print("End Point:", waypoints[-1])

        # Compute Cartesian path
        cartesian_path = np.linspace(waypoints[0], waypoints[-1], num=10)
        print("Computed Cartesian Path:")
        for point in cartesian_path:
            print(point)

    # Create Open3D visualization for point cloud and path
    point_cloud = o3d.geometry.PointCloud()
    path = o3d.geometry.LineSet()

    if waypoints is not None:
        point_cloud.points = o3d.utility.Vector3dVector(waypoints)
        path.points = o3d.utility.Vector3dVector(waypoints)
        path.lines = o3d.utility.Vector2iVector(np.array([[0, 1]]))

    o3d.visualization.draw_geometries([point_cloud, path])

if __name__ == "__main__":
    main()


