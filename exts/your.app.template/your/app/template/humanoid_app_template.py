import carb
import omni.usd
from pxr import Usd, Gf, UsdGeom

import omni.kit.commands
import omni.graph.core as og

from isaacsim.core.utils.prims import create_prim
from isaacsim.core.utils.stage import get_next_free_path
from isaacsim.storage.native import get_assets_root_path
import isaacsim.core.api.objects as objects


def load_stage():
    # load warehouse scene
    create_prim(
        prim_path=get_next_free_path("/World/scene", None),
        prim_type="Xform",
        usd_path="https://omniverse-content-production.s3-us-west-2.amazonaws.com/Assets/ArchVis/Industrial/Buildings/Warehouse/Warehouse01.usd",
        translation=Gf.Vec3d(0, 0, 0),
    )

    # add ground
    objects.GroundPlane("/World/ground_plane", visible=True)


def load_humanoid(humanoid_path:str = "/World/humanoid"):
    """
    Docstring for load_humanoid
    :type humanoid_path: str
    """
    # get stage
    stage = omni.usd.get_context().get_stage()
    humanoid_path = get_next_free_path(humanoid_path, None)
    create_prim(
        prim_path=humanoid_path,
        prim_type="Xform",
        usd_path="https://omniverse-content-production.s3-us-west-2.amazonaws.com/Assets/Isaac/5.1/Isaac/Robots/Unitree/G1/g1.usd",
        translation=Gf.Vec3d(0, 0, 0),
    )
    # move humanoid to be on the ground
    min_box, max_box = omni.usd.get_context().compute_path_world_bounding_box(humanoid_path)
    stage.GetPrimAtPath(humanoid_path).GetAttribute("xformOp:translate").Set(Gf.Vec3d(0, 0, -min_box[2]))
    print(f"Humanoid loaded at {humanoid_path}", min_box[2])
    return humanoid_path


def create_camera(camera_path: str = "/World/camera",
                  position = Gf.Vec3d(0, 0, 0.5),
                  orientation = Gf.Quatd(0.9238795, 0, -0.3826834, 0),
                  enable_ros2_bridge: bool = True):
    """
    Docstring for create_camera
    :type camera_path: str
    """
    # get stage
    # stage = omni.usd.get_context().get_stage()
    camera_path = get_next_free_path(camera_path, None)
    camera_prim = create_prim(
        prim_path=camera_path,
        prim_type="Camera",
        translation=position,
    )
    # set camera orientation
    camera_prim.GetAttribute("xformOp:orient").Set(orientation)  # 90 degrees around X axis
    print(f"Camera created at {camera_prim.GetPath()}")

    if enable_ros2_bridge:
        keys = og.Controller.Keys
        stage = omni.usd.get_context().get_stage()
        og_path = omni.usd.get_stage_next_free_path(stage, f"/Graph/{camera_path.split('/')[-1]}", True)

        keys_CREATE_NODES = [
            ("OnPlaybackTick", "omni.graph.action.OnPlaybackTick"),
            ("RunOnce", "isaacsim.core.nodes.OgnIsaacRunOneSimulationFrame"),
            ("Context", "isaacsim.ros2.bridge.ROS2Context"),

            ("RenderProduct", "isaacsim.core.nodes.IsaacCreateRenderProduct"),
            ("RGBPublish", "isaacsim.ros2.bridge.ROS2CameraHelper"),
            ("CameraInfoPublish", "isaacsim.ros2.bridge.ROS2CameraInfoHelper"),
        ]

        keys_SET_VALUES =  [
            ("RenderProduct.inputs:cameraPrim", f"{camera_path}"),
            ("RenderProduct.inputs:height", 1200),
            ("RenderProduct.inputs:width", 1920),
            
            # camera
            ("CameraInfoPublish.inputs:topicName", "camera_info_left"),
            ("CameraInfoPublish.inputs:frameId", "sim_camera"),

            ("CameraInfoPublish.inputs:resetSimulationTimeOnStop", True),
            ("RGBPublish.inputs:frameId", "sim_camera"),
            ("RGBPublish.inputs:nodeNamespace", "/isaac"),
            ("RGBPublish.inputs:resetSimulationTimeOnStop", True),
            ("RGBPublish.inputs:topicName", f"{camera_path.split('/')[-1]}_rgb"),
        ]

        keys_CONNECT = [
            ("OnPlaybackTick.outputs:tick", "RunOnce.inputs:execIn"),
            ("RunOnce.outputs:step", "RenderProduct.inputs:execIn"),
            ("RenderProduct.outputs:execOut", "CameraInfoPublish.inputs:execIn"),
            ("RenderProduct.outputs:execOut", "RGBPublish.inputs:execIn"),
            ("RenderProduct.outputs:renderProductPath", "CameraInfoPublish.inputs:renderProductPath"),
            ("RenderProduct.outputs:renderProductPath", "RGBPublish.inputs:renderProductPath"),
            ("Context.outputs:context", "CameraInfoPublish.inputs:context"),
            ("Context.outputs:context", "RGBPublish.inputs:context"),
        ]

        (graph_handle, nodes, _, _) = og.Controller.edit(
            {"graph_path": og_path, "evaluator_name": "execution"},
            {
                keys.CREATE_NODES: keys_CREATE_NODES,
                keys.SET_VALUES: keys_SET_VALUES, 
                keys.CONNECT: keys_CONNECT
            },
        )

    return camera_path


def create_lidar(
    lidar_path: str = "/World/lidar",
    asset_path: str = "/Isaac/Sensors/Ouster/OS0/OS0.usd",
    position = Gf.Vec3d(1.0, 0, 0),
    enable_ros2_bridge: bool = True,
):
    """
    Creates a lidar sensor in the scene.
    """
    lidar_path = get_next_free_path(lidar_path, None)
    lidar_prim = create_prim(
        prim_path=lidar_path,
        prim_type="Xform",
        usd_path=get_assets_root_path() + asset_path,
        translation=position,
    )

    print(f"Lidar created at {lidar_prim.GetPath()}")

    # enable ros2 bridge 
    if enable_ros2_bridge:
        keys = og.Controller.Keys
        stage = omni.usd.get_context().get_stage()
        og_path = omni.usd.get_stage_next_free_path(stage, f"/Graph/{lidar_path.split('/')[-1]}", True)

        keys_CREATE_NODES = [
            ("OnPlaybackTick", "omni.graph.action.OnPlaybackTick"),
            ("RenderProduct", "isaacsim.core.nodes.IsaacCreateRenderProduct"),
            ("RunOnce", "isaacsim.core.nodes.OgnIsaacRunOneSimulationFrame"),
            ("Context", "isaacsim.ros2.bridge.ROS2Context"),
            ("LidarPublish", "isaacsim.ros2.bridge.ROS2RtxLidarHelper"),
        ]

        keys_SET_VALUES = [
            ("RenderProduct.inputs:cameraPrim", f"{lidar_path}/sensor"),

            # lidar
            ("LidarPublish.inputs:topicName", "lidar_points"),
            ("LidarPublish.inputs:frameId", "sim_lidar"),

            ("LidarPublish.inputs:nodeNamespace", "/isaac"),
            ("LidarPublish.inputs:resetSimulationTimeOnStop", True),
            ("LidarPublish.inputs:showDebugView", True),
        ]

        keys_CONNECT = [
            ("OnPlaybackTick.outputs:tick", "RunOnce.inputs:execIn"),
            ("RunOnce.outputs:step", "RenderProduct.inputs:execIn"),
            ("RenderProduct.outputs:execOut", "LidarPublish.inputs:execIn"),
            ("RenderProduct.outputs:renderProductPath", "LidarPublish.inputs:renderProductPath"),
            ("Context.outputs:context", "LidarPublish.inputs:context"),
        ]

        (graph_handle, nodes, _, _) = og.Controller.edit(
            {"graph_path": og_path, "evaluator_name": "execution"},
            {
                keys.CREATE_NODES: keys_CREATE_NODES,
                keys.SET_VALUES: keys_SET_VALUES, 
                keys.CONNECT: keys_CONNECT
                },
        )


    return lidar_path


def start_simulation():
    """
    Write your workflow code here.
    """
    pass

if __name__ == "__main__":
    # Example usage
    load_stage()
    humanoid_path = load_humanoid()
    create_camera(
            camera_path="/World/humanoid/torso_link/head_link/camera_left",
            position=Gf.Vec3d(0, 0.05, 0.5),
            orientation=Gf.Quatd(0.9238795, 0, -0.3826834, 0),
            enable_ros2_bridge=True
        )

    create_camera(
        camera_path="/World/humanoid/torso_link/head_link/camera_right",
        position=Gf.Vec3d(0, -0.05, 0.5),
        orientation=Gf.Quatd(0.9238795, 0, -0.3826834, 0),
        enable_ros2_bridge=True
        )
    
    lidar_path = create_lidar()
    
    carb.log_info("Setup complete. You can now start the simulation.")