#!/usr/bin/env python

import problem
import heapq
import rospy
import sys
import time
import re
import json
from std_msgs.msg import String

root_path = "/home/aravamuthan/catkin_ws/src/Planning/";
bot = 1;

def is_goal_state(state, g_s):
    if state.x == g_s.x and state.y == g_s.y:
        return 1;
    return 0;

def manhattanDistance(x1, y1, x2, y2):
    """
    This function returns manhattan distance between two points.
    """
    return abs(x1 - x2) + abs(y1 - y2)


def stringifyState(state):
    return str(state.x) + str(state.y) + str(state.orientation);


def manhattanHeuristics(sourceState, destinationState):
    return abs(sourceState.x - destinationState.x) + abs(sourceState.y - destinationState.y);


def gbfs(i_s, g_s):
    tic = time.clock();
    # init_state = problem.get_initial_state();
    # goal_state = problem.get_goal_state();
    init_state = i_s;
    goal_state = g_s;
    print("init state", stringifyState(init_state));
    print("goal state", stringifyState(goal_state));
    possible_actions = problem.get_actions()
    action_list = [];
    if (is_goal_state(init_state, g_s)):
        return action_list, init_state;
    explored_states = {};
    frontier = [(manhattanHeuristics(init_state, goal_state), init_state, action_list)];
    while frontier:
        [current_cost, current_state, current_path] = heapq.heappop(frontier);
        if (is_goal_state(current_state, g_s)):
            print("goal found " + stringifyState(current_state));
            print(len(current_path));
            toc = time.clock();
            print(toc - tic);
            return current_path, current_state;
        for possible_action in possible_actions:
            (nextstate, cost) = problem.get_successor(current_state, possible_action);
            if stringifyState(nextstate) not in explored_states and cost > 0:
                path_e = current_path[:];
                path_e.append(possible_action);
                # print(stringifyState(current_state), stringifyState(nextstate), possible_action);
                heapq.heappush(frontier, (manhattanHeuristics(nextstate, goal_state), nextstate, path_e));
        explored_states[stringifyState(current_state)] = current_cost;
    toc = time.clock();
    print(toc - tic);
    print("goal not found");
    return [], init_state;

def readPlan(fileName):
    with open(fileName + '.txt') as file:
        file_contents = file.read();
        actions = re.findall(r"\(([A-Za-z0-9_ ]+)\)", file_contents);
        return actions;



def readJson(fileName):
    with open(fileName + '.json') as json_file:
        data = json.load(json_file);
        # print(data);
        return data;

def driverFunction(actions, books, bins):
    i_s = problem.get_initial_state();
    for action in actions:
        strs = action.split();
        if strs[0] == "move":
            goal = strs[3].split("_iloc")[0];
            if goal.find("book") > -1:
                goalLocation = books[goal]['load_loc'][0];
                print(goalLocation[0])
                print(goalLocation[1])
                g_s = problem.State(goalLocation[0],goalLocation[1],"EAST");
            elif goal.find("trolly") > -1:
                goalLocation = bins[goal]['load_loc'][0];
                g_s = problem.State(goalLocation[0], goalLocation[1], "EAST");
            path, current_state = gbfs(i_s, g_s);
            problem.execute_move_action(path);
            i_s = current_state;
            bot = 1;
            while(bot):
                print("bot busy");
            print("bot free");
        elif strs[0] == "pick":
            print("Picking bock " +  strs[1] + " from " + stringifyState(i_s));
            op = problem.execute_pick_action(strs[1], i_s);
            print(op);
            if op < 0:
                print("counld'nt pick ");
                return;
            bot = 1;
            while (bot):
                print("bot busy");
            print("bot free");
        elif strs[0] == "place":
            print("Placing bock " + strs[1] + " at " + stringifyState(i_s));
            print("Placing book at bin " + strs[2]);
            op = problem.execute_place_action(strs[1], strs[2], i_s);
            print(op);
            if op < 0:
                print("counld'nt place ");
                return;
            bot = 1;
            while (bot):
                print("bot busy");
            print("bot free");
        else:
            continue;


def callback_pid(data):
    if data.data == "Idle":
        bot = 0;


if __name__ == "__main__":
    status_subscriber = rospy.Subscriber("/Controller_Status", String, callback_pid);
    actions = readPlan("PlanH");
    jsonData = readJson(root_path + '/books');
    books = jsonData['books'];
    bins = jsonData['bins'];
    #print(books);
    #print(bins);
    driverFunction(actions, books, bins);
