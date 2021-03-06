#!/usr/bin/env python

import problem
import heapq
import rospy
import time
import re
import json
from std_msgs.msg import String
import matplotlib.pyplot as plt

root_path = "/home/aravamuthan/catkin_ws/src/Planning/";

bot = 1;
count = 0;

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
            return current_path, g_s;
        for possible_action in possible_actions:
            (nextstate, cost) = problem.get_successor(current_state, possible_action);
            if stringifyState(nextstate) not in explored_states and cost > 0:
                path_e = current_path[:];
                path_e.append(possible_action);
                # print(stringifyState(current_state), stringifyState(nextstate), possible_action);
                heapq.heappush(frontier, (manhattanHeuristics(nextstate, goal_state), nextstate, path_e));
        explored_states[stringifyState(current_state)] = current_cost;
    toc = time.clock();
    #print(toc - tic);
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
            print("moving");
            goal = strs[3].split("_iloc")[0];
            if goal.find("book") > -1:
                goalLocation = books[goal]['load_loc'][0];
                g_s = problem.State(goalLocation[0],goalLocation[1],"EAST");
            elif goal.find("trolly") > -1:
                goalLocation = bins[goal]['load_loc'][0];
                g_s = problem.State(goalLocation[0], goalLocation[1], "EAST");
            path, current_state = gbfs(i_s, g_s);
            problem.execute_move_action(path);
            i_s = current_state;
            time.sleep(1);
            global count;
            count = count + 1;
        elif strs[0] == "pick":
            print("Picking bock " +  strs[1] + " from " + stringifyState(i_s));
            op = problem.execute_pick_action(strs[1], i_s);
            if op < 0:
                print("counld'nt pick ");
                return;
            time.sleep(1);
            global count;
            count = count + 1;
        elif strs[0] == "place":
            print("Placing bock " + strs[1] + " at " + stringifyState(i_s));
            print("Placing book at bin " + strs[2]);
            op = problem.execute_place_action(strs[1], strs[2], i_s);
            if op < 0:
                print("counld'nt place ");
                return;
            time.sleep(1);
            global count;
            count = count + 1;
        else:
            continue;


def callback(data):
    if data.data == "Idle":
        global bot;
        bot = 0;
        print("bot Idle");

def plotResults():
    time_in_seconds_with_delay_for_idling = [17.32, 27.05, 37.60, 47.15, 58.23, 69.75];
    time_in_seconds_without_delay_for_idling = [1.21, 3.02, 4.60, 6.48, 8.20, 12.85];
    x_axis = [2, 3, 4, 5, 6, 7];
    plt.plot(x_axis, time_in_seconds_with_delay_for_idling, label="With idle Subcription");
    plt.plot(x_axis, time_in_seconds_without_delay_for_idling, label="Without idle Subcription");
    plt.legend(loc='upper left');
    plt.xlabel('subjects');
    plt.ylabel("time taken(sec)");
    plt.show();



if __name__ == "__main__":
    tic = time.clock();
    rospy.init_node('reach_goal', anonymous=True);
    status_subscriber = rospy.Subscriber("/status", String, callback);
    n = 0;
    while n < 7:
	    strn = str(n+1)  
	    actions = readPlan("PlanH" + strn);
	    jsonData = readJson(root_path + '/books');
	    books = jsonData['books'];
	    bins = jsonData['bins'];
	    driverFunction(actions, books, bins);
	    toc = time.clock();
	    print("no. of subjects " + strn);
	    print("take taken: ");
	    print(toc - tic);
	    print("take taken with ")
	    print(count)
	    print("sleeps is")
	    print(toc - tic + count);
	    n = n +1;	
    plotResults();


