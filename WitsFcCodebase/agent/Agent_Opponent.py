from agent.Base_Agent import Base_Agent
from math_ops.Math_Ops import Math_Ops as M
import math
import numpy as np

from strategy.Assignment import role_assignment 
from strategy.Strategy import Strategy 

from formation.Formation import GenerateBasicFormation


def pass_reciever_selector(player_unum, teammate_positions, goal_position):
    """
    Simple pass receiver selector - picks the next player in line
    """
    pass_reciever_unum = player_unum + 1
    if pass_reciever_unum > 5:
        pass_reciever_unum = 1
    
    # Return the position of the pass receiver
    target = teammate_positions[pass_reciever_unum-1]
    if target is None:
        return goal_position
    return target


class Agent(Base_Agent):
    def __init__(self, host:str, agent_port:int, monitor_port:int, unum:int,
                 team_name:str, enable_log, enable_draw, wait_for_server=True, is_fat_proxy=False) -> None:
        
        # define robot type
        robot_type = (0,1,1,1,2,3,3,3,4,4,4)[unum-1]

        # Initialize base agent
        # Args: Server IP, Agent Port, Monitor Port, Uniform No., Robot Type, Team Name, Enable Log, Enable Draw, play mode correction, Wait for Server, Hear Callback
        super().__init__(host, agent_port, monitor_port, unum, robot_type, team_name, enable_log, enable_draw, True, wait_for_server, None)

        self.enable_draw = enable_draw
        self.state = 0  # 0-Normal, 1-Getting up, 2-Kicking
        self.kick_direction = 0
        self.kick_distance = 0
        self.fat_proxy_cmd = "" if is_fat_proxy else None
        self.fat_proxy_walk = np.zeros(3) # filtered walk parameters for fat proxy

        self.init_pos = ([-14,0],[-9,-5],[-9,0],[-9,5],[-5,-5],[-5,0],[-5,5],[-1,-6],[-1,-2.5],[-1,2.5],[-1,6])[unum-1] # initial formation


    def beam(self, avoid_center_circle=False):
        r = self.world.robot
        pos = self.init_pos[:] # copy position list 
        self.state = 0

        # Avoid center circle by moving the player back 
        if avoid_center_circle and np.linalg.norm(self.init_pos) < 2.5:
            pos[0] = -2.3 

        if np.linalg.norm(pos - r.loc_head_position[:2]) > 0.1 or self.behavior.is_ready("Get_Up"):
            self.scom.commit_beam(pos, 0)
            for _ in range(7):
                self.scom.commit_and_send( r.get_command() )
                self.scom.receive()
        else:
            self.behavior.execute("Zero_Bent_Knees")


    def fat_proxy_kick(self):
        """
        Kick using Fat Proxy commands
        """
        self.fat_proxy_cmd = f"kick {self.kick_distance} {self.kick_direction}"
        return True


    def move(self, target_2d, is_target_absolute=True, orientation=None, is_orientation_absolute=True, 
             distance=None, avoid_obstacles=True, priority_unums=[], is_aggressive=False, timeout=20):
        """
        Walk to target with optional path planning
        """
        r = self.world.robot

        # Fat proxy movement
        if self.fat_proxy_cmd is not None:
            target_rel = M.target_abs_to_rel(target_2d, r.loc_head_position[:2], r.imu_torso_orientation) if is_target_absolute else target_2d
            ori_rel = orientation - r.imu_torso_orientation if is_orientation_absolute and orientation is not None else (orientation if orientation is not None else 0)
            
            alpha = 0.3
            self.fat_proxy_walk = self.fat_proxy_walk * (1-alpha) + np.array([target_rel[0], target_rel[1], ori_rel]) * alpha

            if np.linalg.norm(self.fat_proxy_walk[:2]) < 0.01:
                self.fat_proxy_walk[:2] = 0
            if abs(self.fat_proxy_walk[2]) < 0.5:
                self.fat_proxy_walk[2] = 0

            self.fat_proxy_cmd = f"move {self.fat_proxy_walk[0]} {self.fat_proxy_walk[1]} {self.fat_proxy_walk[2]}"
            return

        # Normal movement
        if not is_target_absolute:
            target_2d = r.loc_head_position[:2] + target_2d
        
        if not is_orientation_absolute and orientation is not None:
            orientation += r.imu_torso_orientation

        if not avoid_obstacles:
            distance_to_final_target = np.linalg.norm(target_2d - r.loc_head_position[:2])
            self.behavior.execute("Walk", target_2d, True, orientation, is_orientation_absolute, distance_to_final_target)
            return

        if avoid_obstacles:
            target_2d, _, distance_to_final_target = self.path_manager.get_path_to_target(
                target_2d, priority_unums=priority_unums, is_aggressive=is_aggressive, timeout=timeout)
        else:
            distance_to_final_target = np.linalg.norm(target_2d - r.loc_head_position[:2])

        self.behavior.execute("Walk", target_2d, True, orientation, is_orientation_absolute, distance_to_final_target)


    def kick(self, kick_direction=None, kick_distance=None, abort=False, enable_pass_command=False):
        """
        Walk to ball and kick with maximum power
        """
        self.kick_direction = self.kick_direction if kick_direction is None else kick_direction
        self.kick_distance = self.kick_distance if kick_distance is None else kick_distance

        # Always use fat proxy kick for maximum power
        if self.fat_proxy_cmd is not None:
            return self.fat_proxy_kick()
        else:
            # If not in fat proxy mode, use Basic_Kick
            return self.behavior.execute("Basic_Kick", self.kick_direction, abort)


    def kickTarget(self, strategyData, mypos_2d=(0,0), target_2d=(0,0), abort=False, enable_pass_command=False):
        """
        Walk to ball and kick toward target

        Parameters
        ----------
        strategyData : Strategy object
            Contains game state information
        mypos_2d : tuple
            Current position
        target_2d : tuple
            Target position to kick toward
        abort : bool
            True to abort
        enable_pass_command : bool
            When True, the pass command will be used when at least one opponent is near the ball
            
        Returns
        -------
        finished : bool
            Returns True if the behavior finished or was successfully aborted.
        '''
        """
        # Calculate the vector from the current position to the target position
        vector_to_target = np.array(target_2d) - np.array(mypos_2d)
        
        # Calculate the distance (magnitude of the vector)
        kick_distance = np.linalg.norm(vector_to_target)
        
        # Calculate the direction (angle) in radians
        direction_radians = np.arctan2(vector_to_target[1], vector_to_target[0])
        
        # Convert direction to degrees
        kick_direction = np.degrees(direction_radians)

        if strategyData.min_opponent_ball_dist < 1.45 and enable_pass_command:
            self.scom.commit_pass_command()

        self.kick_direction = kick_direction
        self.kick_distance = kick_distance

        if self.fat_proxy_cmd is None: # normal behavior
            return self.behavior.execute("Basic_Kick", self.kick_direction, abort)
        else: # fat proxy behavior
            return self.fat_proxy_kick()


    def think_and_send(self):
        
        behavior = self.behavior
        strategyData = Strategy(self.world)
        d = self.world.draw

        if strategyData.play_mode == self.world.M_GAME_OVER:
            pass
        elif strategyData.PM_GROUP == self.world.MG_ACTIVE_BEAM:
            self.beam()
        elif strategyData.PM_GROUP == self.world.MG_PASSIVE_BEAM:
            self.beam(True) # avoid center circle
        elif self.state == 1 or (behavior.is_ready("Get_Up") and self.fat_proxy_cmd is None):
            self.state = 0 if behavior.execute("Get_Up") else 1
        else:
            if strategyData.play_mode != self.world.M_BEFORE_KICKOFF:
                self.select_skill(strategyData)
            else:
                pass

        #--------------------------------------- 3. Broadcast
        self.radio.broadcast()

        #--------------------------------------- 4. Send to server
        if self.fat_proxy_cmd is None: # normal behavior
            self.scom.commit_and_send( strategyData.robot_model.get_command() )
        else: # fat proxy behavior
            self.scom.commit_and_send( self.fat_proxy_cmd.encode() ) 
            self.fat_proxy_cmd = ""


    def select_skill(self, strategyData):
        """
        Default strategy: Role Assignment + Pass Selector
        Clean version without print statements
        """
        drawer = self.world.draw
        
        # Role Assignment
        if strategyData.active_player_unum == strategyData.robot_model.unum:
            drawer.annotation((0,10.5), "Role Assignment Phase", drawer.Color.yellow, "status")
        else:
            drawer.clear("status")
        
        formation_positions = GenerateBasicFormation()
        point_preferences = role_assignment(strategyData.teammate_positions, formation_positions)
        
        if strategyData.player_unum in point_preferences:
            strategyData.my_desired_position = point_preferences[strategyData.player_unum]
            strategyData.my_desried_orientation = strategyData.GetDirectionRelativeToMyPositionAndTarget(strategyData.my_desired_position)
            drawer.line(strategyData.mypos, strategyData.my_desired_position, 2, drawer.Color.blue, "target line")
        
        if not strategyData.IsFormationReady(point_preferences):
            return self.move(strategyData.my_desired_position, orientation=strategyData.my_desried_orientation)
        
        # Pass Selector
        if strategyData.active_player_unum == strategyData.robot_model.unum:
            drawer.annotation((0,10.5), "Pass Selector Phase", drawer.Color.yellow, "status")
            target = pass_reciever_selector(strategyData.player_unum, strategyData.teammate_positions, (15,0))
            drawer.line(strategyData.mypos, target, 2, drawer.Color.red, "pass line")
            return self.kickTarget(strategyData, strategyData.mypos, target)
        else:
            drawer.clear("pass line")
            return self.move(strategyData.my_desired_position, orientation=strategyData.ball_dir)