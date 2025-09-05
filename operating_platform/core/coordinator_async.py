
import cv2
import json
import time
import draccus
import socketio
import requests
import traceback
import threading

from dataclasses import dataclass, asdict
from pathlib import Path
from pprint import pformat
from deepdiff import DeepDiff
from functools import cache
from termcolor import colored
from datetime import datetime


# from operating_platform.policy.config import PreTrainedConfig
from operating_platform.robot.robots.configs import RobotConfig
from operating_platform.robot.robots.utils import make_robot_from_config, Robot, busy_wait, safe_disconnect
from operating_platform.utils import parser
from operating_platform.utils.utils import has_method, init_logging, log_say, get_current_git_branch, git_branch_log

from operating_platform.utils.constants import DOROBOT_DATASET
from operating_platform.dataset.dorobot_dataset import *

# from operating_platform.core._client import Coordinator
from operating_platform.core.daemon import Daemon
from operating_platform.core.record import Record, RecordConfig
import asyncio  
import aiohttp

DEFAULT_FPS=20





@cache
def is_headless():
    """Detects if python is running without a monitor."""
    try:
        import pynput  # noqa

        return False
    except Exception:
        print(
            "Error trying to import pynput. Switching to headless mode. "
            "As a result, the video stream from the cameras won't be shown, "
            "and you won't be able to change the control flow with keyboards. "
            "For more info, see traceback below.\n"
        )
        traceback.print_exc()
        print()
        return True

# def init_keyboard_listener():
#     # Allow to exit early while recording an episode or resetting the environment,
#     # by tapping the right arrow key '->'. This might require a sudo permission
#     # to allow your terminal to monitor keyboard events.
#     events = {}
#     events["exit_early"] = False
#     events["rerecord_episode"] = False
#     events["stop_recording"] = False

#     if is_headless():
#         logging.warning(
#             "Headless environment detected. On-screen cameras display and keyboard inputs will not be available."
#         )
#         listener = None
#         return listener, events

#     # Only import pynput if not in a headless environment
#     from pynput import keyboard

#     def on_press(key):
#         try:
#             if key == keyboard.Key.right:
#                 print("Right arrow key pressed. Exiting loop...")
#                 events["exit_early"] = True
#             elif key == keyboard.Key.left:
#                 print("Left arrow key pressed. Exiting loop and rerecord the last episode...")
#                 events["rerecord_episode"] = True
#                 events["exit_early"] = True
#             elif key == keyboard.Key.esc:
#                 print("Escape key pressed. Stopping data recording...")
#                 events["stop_recording"] = True
#                 events["exit_early"] = True
#             elif key.char == 'q' or key.char == 'Q':  # 检测q键（不区分大小写）
#                 print("Q key pressed.")
#                 events["exit_early"] = True

#         except Exception as e:
#             print(f"Error handling key press: {e}")

#     listener = keyboard.Listener(on_press=on_press)
#     listener.start()

#     return listener, events


def cameras_to_stream_json(cameras: dict[str, int]):
    """
    将摄像头字典转换为包含流信息的 JSON 字符串。
    
    参数:
        cameras (dict[str, int]): 摄像头名称到 ID 的映射
    
    返回:
        str: 格式化的 JSON 字符串
    """
    stream_list = [{"id": cam_id, "name": name} for name, cam_id in cameras.items()]
    result = {
        "total": len(stream_list),
        "streams": stream_list
    }
    return json.dumps(result)

class Coordinator:
    def __init__(self, daemon: Daemon, server_url="http://localhost:8088"):
        self.server_url = server_url
        self.sio = socketio.AsyncClient()  
        self.session = aiohttp.ClientSession(  
            connector=aiohttp.TCPConnector(limit=10, limit_per_host=10)  
        ) 

        self.daemon = daemon

        self.running = False
        self.last_heartbeat_time = 0
        self.heartbeat_interval = 2  # 心跳间隔(秒)

        self.recording = False
        self.saveing = False


        self.cameras: dict[str, int] = {
            "image_left": 1,
            "image_right": 2,
            "image_external":3
        } # Default Config
        
        # 注册事件处理
        self.sio.on('HEARTBEAT_RESPONSE', self.__on_heartbeat_response_handle)  
        self.sio.on('connect', self.__on_connect_handle)  
        self.sio.on('disconnect', self.__on_disconnect_handle)  
        self.sio.on('robot_command', self.__on_robot_command_handle)  

        self.record = None
    
####################### Client Start/Stop ############################
    async def start(self):  
        """启动客户端"""  
        self.running = True  
        await self.sio.connect(self.server_url)  
        # 使用 asyncio 任务发送心跳  
        asyncio.create_task(self.send_heartbeat_loop())
        
        # print("客户端已启动，等待连接...")
    
    async def stop(self):  
        """停止客户端"""  
        self.running = False  
        await self.sio.disconnect()  
        await self.session.close()  
        print("客户端已停止")
    
####################### Client Handle ############################
    async def __on_heartbeat_response_handle(self, data):
        """心跳响应回调"""
        print("收到心跳响应:", data)
    
    async def __on_connect_handle(self):
        """连接成功回调"""
        print("成功连接到服务器")
        
        # # 初始化视频流列表
        # try:
        #     response = self.session.post(
        #         f"{self.server_url}/robot/stream_info",
        #         json = cameras_to_stream_json(self.cameras),
        #     )
        #     print("初始化视频流列表:", response.json())
        # except Exception as e:
        #     print(f"初始化视频流列表失败: {e}")
    
    async def __on_disconnect_handle(self):
        """断开连接回调"""
        print("与服务器断开连接")
    
    async def __on_robot_command_handle(self, data):
        """收到机器人命令回调"""
        print("收到服务器命令:", data)
        
        # 根据命令类型进行响应
        if data.get('cmd') == 'video_list':
            print("处理更新视频流命令...")
            response_data = cameras_to_stream_json(self.cameras)
            # 发送响应
            try:
                response = self.session.post(
                    f"{self.server_url}/robot/stream_info",
                    json = response_data,
                )
                print(f"已发送响应 [{data.get('cmd')}]: {response_data}")
            except Exception as e:
                print(f"发送响应失败 [{data.get('cmd')}]: {e}")
            
        elif data.get('cmd') == 'start_collection':
            print("处理开始采集命令...")
            msg = data.get('msg')

            if self.recording == True:
                # self.send_response('start_collection', "fail")

                self.record.stop()
                self.record.discard()
                self.recording = False


            self.recording = True

            task_id = msg.get('task_id')
            task_name = msg.get('task_name')
            task_data_id = msg.get('task_data_id')
            repo_id=f"{task_name}_{task_id}"

            date_str = datetime.now().strftime("%Y%m%d")

            # 构建目标目录路径
            dataset_path = DOROBOT_DATASET

            git_branch_name = get_current_git_branch()
            if git_branch_name and "release" in git_branch_name:
                target_dir = dataset_path / date_str / "user" / repo_id
            elif git_branch_name and "dev"  in git_branch_name:
                target_dir = dataset_path / date_str / "dev" / repo_id
            else:
                target_dir = dataset_path / date_str / "dev" / repo_id


            # 判断是否存在对应文件夹以决定是否启用恢复模式
            resume = False

            # 检查数据集目录是否存在
            if not dataset_path.exists():
                logging.info(f"Dataset directory '{dataset_path}' does not exist. Cannot resume.")
            else:
                # 检查目标文件夹是否存在且为目录
                if target_dir.exists() and target_dir.is_dir():
                    resume = True
                    logging.info(f"Found existing directory for repo_id '{repo_id}'. Resuming operation.")
                else:
                    logging.info(f"No directory found for repo_id '{repo_id}'. Starting fresh.")

            # resume 变量现在可用于后续逻辑
            print(f"Resume mode: {'Enabled' if resume else 'Disabled'}")
            #！！！
            record_cfg = RecordConfig(fps=DEFAULT_FPS, repo_id=repo_id, resume=resume, root=target_dir)
            self.record = Record(fps=DEFAULT_FPS, robot=self.daemon.robot, daemon=self.daemon, record_cfg = record_cfg, record_cmd=msg)
            
            self.record.start()

            # 发送响应
            await self.send_response('start_collection', "success")
        
        elif data.get('cmd') == 'finish_collection':
            print("处理完成采集命令...")

            if not self.saveing and self.record.save_data is None:
                # 如果不在保存状态，立即停止记录并保存
                self.saveing= True
                self.record.stop()
                self.record.save()
                self.recording = False
                self.saveing= False

            # 如果正在保存，循环等待直到 self.record.save_data 有数据
            while self.saveing:
                time.sleep(0.1)  # 避免CPU过载，适当延迟

            # 此时无论 saveing 状态如何，self.record.save_data 已有有效数据
            response_data = {
                "msg": "success",
                "data": self.record.save_data,
            }

            # 发送响应
            await self.send_response('finish_collection', response_data['msg'], response_data)
        
        elif data.get('cmd') == 'discard_collection':
            # 模拟处理丢弃采集
            print("处理丢弃采集命令...")

            self.record.stop()
            self.record.discard()
            self.recording = False

            # 发送响应
            await self.send_response('discard_collection', "success")
        
        elif data.get('cmd') == 'submit_collection':
            # 模拟处理提交采集
            print("处理提交采集命令...")
            time.sleep(0.01)  # 模拟处理时间
            
            # 发送响应
            await self.send_response('submit_collection', "success")
    
####################### Client Send to Server ############################
    async def send_heartbeat_loop(self):  
        """定期发送心跳"""  
        while self.running:  
            current_time = time.time()  
            if current_time - self.last_heartbeat_time >= self.heartbeat_interval:  
                try:  
                    await self.sio.emit('HEARTBEAT')  
                    self.last_heartbeat_time = current_time  
                except Exception as e:  
                    print(f"发送心跳失败: {e}")  
            await asyncio.sleep(1)

    # 发送回复请求
    async def send_response(self, cmd, msg, data=None):  
        """发送回复请求到服务器"""  
        payload = {"cmd": cmd, "msg": msg}  
        if data:  
            payload.update(data)  
        try:  
            async with self.session.post(  
                f"{self.server_url}/robot/response",  
                json=payload,  
                timeout=aiohttp.ClientTimeout(total=2)  
            ) as resp:  
                print(f"已发送响应 [{cmd}]: {payload}")  
        except Exception as e:  
            print(f"发送响应失败 [{cmd}]: {e}")

####################### Robot API ############################
    def stream_info(self, info: dict[str, int]):
        self.cameras = info.copy()
        print(f"更新摄像头信息: {self.cameras}")

    async def update_stream_info_to_server(self):  
        stream_info_data = cameras_to_stream_json(self.cameras)  
        print(f"stream_info_data: {stream_info_data}")  
        try:  
            async with self.session.post(  
                f"{self.server_url}/robot/stream_info",  
                json=stream_info_data,  
                timeout=aiohttp.ClientTimeout(total=2)  
            ) as response:  
                if response.status == 200:  
                    print("摄像头流信息已同步到服务器")  
                else:  
                    print(f"同步流信息失败: {response.status}")  
        except Exception as e:  
            print(f"同步流信息异常: {e}")

    async def update_stream_async(self, name, frame):  
        """异步视频流传输"""  
        _, jpeg_frame = cv2.imencode('.jpg', frame,   
                            [int(cv2.IMWRITE_JPEG_QUALITY), 80])  
        frame_data = jpeg_frame.tobytes()  
        
        stream_id = self.cameras[name]  
        url = f"{self.server_url}/robot/update_stream/{stream_id}"  
        
        try:  
            async with self.session.post(  
                url,   
                data=frame_data,  
                timeout=aiohttp.ClientTimeout(total=0.2)  
            ) as resp:  
                if resp.status != 200:  
                    text = await resp.text()  
                    print(f"Server error {resp.status}: {text}")  
        except asyncio.TimeoutError:  
            print("update_stream timeout")  
        except Exception as e:  
            print(f"update_stream exception: {e}")

@dataclass
class ControlPipelineConfig:
    robot: RobotConfig
    # control: ControlConfig

    @classmethod
    def __get_path_fields__(cls) -> list[str]:
        """This enables the parser to load config from the policy using `--policy.path=local/dir`"""
        return ["control.policy"]
    

@parser.wrap()  
def main(cfg: ControlPipelineConfig):  
    asyncio.run(async_main(cfg))

async def async_main(cfg: ControlPipelineConfig):  
    """异步主函数"""  
    init_logging()  
    git_branch_log()  
    logging.info(pformat(asdict(cfg)))  

    daemon = Daemon(fps=DEFAULT_FPS)  
    daemon.start(cfg.robot)  

    coordinator = Coordinator(daemon)  
    await coordinator.start()  

    coordinator.stream_info(daemon.cameras_info)  
    await coordinator.update_stream_info_to_server()  

    try:  
        while True:  
            daemon.update()  
            observation = daemon.get_observation()  
            
            if observation is not None:  
                tasks = []  
                image_keys = [key for key in observation if "image" in key]  
                
                for key in image_keys:  
                    img = cv2.cvtColor(observation[key].numpy(), cv2.COLOR_RGB2BGR)  
                    name = key[len("observation.images."):]  
                    
                    # 添加异步视频流任务  
                    tasks.append(coordinator.update_stream_async(name, img))  
                    
                    if not is_headless():  
                        cv2.imshow(name, img)  
                        cv2.waitKey(1)  
                
                # 并发执行所有视频流传输  
                if tasks:  
                    try:  
                        await asyncio.wait_for(  
                            asyncio.gather(*tasks, return_exceptions=True),  
                            timeout=0.2  
                        )  
                    except asyncio.TimeoutError:  
                        pass  
            
            # 让事件循环可以调度其他任务  
            await asyncio.sleep(0)  
            
    except KeyboardInterrupt:  
        print("coordinator and daemon stop")  
    finally:  
        daemon.stop()  
        await coordinator.stop()  
        cv2.destroyAllWindows()
# @parser.wrap()
# def main(cfg: ControlPipelineConfig):

#     init_logging()
#     git_branch_log()
#     logging.info(pformat(asdict(cfg)))

#     daemon = Daemon(fps=DEFAULT_FPS)
#     daemon.start(cfg.robot)

#     coordinator = Coordinator(daemon)
#     coordinator.start()

#     coordinator.stream_info(daemon.cameras_info)
#     coordinator.update_stream_info_to_server()




#     # frame_counter = 0 
#     try:
#         while True:
#             daemon.update()
#             # print(1.0/(end-start))
#             observation = daemon.get_observation()
#             # print(observation['observation.state.wrist'])
#             # end=time.time()
#             # print(time.time())
#             if observation is not None:
#                 image_keys = [key for key in observation if "image" in key]
#                 for i, key in enumerate(image_keys, start=1):
#                     img = cv2.cvtColor(observation[key].numpy(), cv2.COLOR_RGB2BGR) 

#                     name = key[len("observation.images."):] 
                     
#                     coordinator.update_stream(name, img)  
                    

#                     if not is_headless():
#                         # print(f"will show image, name:{name}")
#                         cv2.imshow(name, img)
#                         cv2.waitKey(1)
#                         # print("show image succese")
#             # if observation is not None:  
#             #     # 直接从 daemon.robot 访问全局数据  
#             #     if hasattr(daemon.robot, '__class__') and daemon.robot.__class__.__name__ == 'DexterousHandManipulator':  
#             #         # 导入模块以访问全局变量  
#             #         from operating_platform.robot.robots.dexterous_hand_v1 import manipulator as dh_module  
                    
#             #         # 检查数据是否存在  
#             #         has_left_wrist = 'wrist_left' in dh_module.recv_wrist_data  
#             #         has_right_wrist = 'wrist_right' in dh_module.recv_wrist_data
#             #         has_left_finger = 'finger_left' in dh_module.recv_finger_data  
#             #         has_right_finger = 'finger_right' in dh_module.recv_finger_data    
#             #         has_head = 'head_pose' in dh_module.recv_head_data  
                    
#             #         if has_left_wrist and has_right_wrist and has_head and has_left_finger and has_right_finger:  
#             #             visualizer = get_visualizer()
            
#             #             visualizer.update_poses(dh_module.recv_wrist_data, dh_module.recv_head_data,dh_module.recv_finger_data)
            
            
                     
#             # else:
#             #     print("observation is none")
            
#     except KeyboardInterrupt:
#         print("coordinator and daemon stop")

#     finally:
#         daemon.stop()
#         coordinator.stop()
#         cv2.destroyAllWindows()
    

if __name__ == "__main__":
    main()
