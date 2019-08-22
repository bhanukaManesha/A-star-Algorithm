import argparse as ap
import re
import platform

######## RUNNING THE CODE ####################################################
#   You can run this code from terminal by executing the following command
#   python planpath.py <INPUT/input#.txt> <OUTPUT/output#.txt> <flag> <algorithm>
#   for example: python planpath.py INPUT/input2.txt OUTPUT/output2.txt 0 A
#   NOTE: THIS IS JUST ONE EXAMPLE INPUT DATA
###############################################################################

import time

########################################################################################################################
########################################################################################################################

class Node:
    '''
    The Node Class that is used to maintain the data for each node in the graph. It also contains the necessary methods
    to print and display the values of the node
    '''
    def __init__(self, identifier, operator, order_of_expansion, cost, heuristic, parent, coordinates):
        '''
        Method used to instantiate a class
        @param identifier: an unique integer value for the node
        @param operator: the operator that generated the node from parent
        @param order_of_expansion: the order in which this node was expanded
        @param cost: the cost to reach this node
        @param heuristic: the estimated cost to reach the goal from this node
        @param parent: reference to the parent node
        @param coordinates: the coordinates of this node on the node map
        '''
        self.identifier = identifier
        self.operator = operator
        self.order_of_expansion = order_of_expansion
        self.cost = cost
        self.heuristic = heuristic
        self.f = cost + heuristic                           # the sum of the actual cost and estimated cost
        self.parent = parent
        self.children = []                                  # the list of child nodes of this node
        self.coordinates = coordinates
        self.isGOAL = False                                 # flag to determine whether its a goal state or not

    def get_operators_to_root(self):
        '''
        This method is used to generate the path from the node to the root(start), traversing through the parent
        @return: the path from the start node as a string
        '''
        current = self
        output = ''
        first = True
        while current != None:
            if current.isGOAL:
                output += '-G'
            if first:
                output = str(current.operator) + output
                first = False
            else:
                output =  str(current.operator) + '-' + output
            current = current.parent
        return output

    def __lt__(self, other):
        '''
        This method is used to override the less than operator in python to use the f cost for comparison
        @param other: the other node to be compared
        @return: boolean value stating whether its less than or not
        '''
        return (self.f < other.f)

    def __len__(self):
        '''
        This method is used to return the depth of the node from the root node. It traverse though the parents to get
        to the root node.
        @return: the depth of the node as an integer
        '''
        count = 0
        current_node = self
        while current_node != None:
            count += 1
            current_node = current_node.parent
        return count - 1

    def __str__(self):
        '''
        This method converts the node into a string for output
        @return: a string with all the data as a string
        '''

        # Converts the child array to a string
        children_str = ''
        if (len(self.children)) > 0:
            for i in range(len(self.children) - 1):
                children_str += 'N' + str(self.children[i].identifier) + ':' + self.children[i].get_operators_to_root() + ','
            children_str += 'N' + str(self.children[-1].identifier) + ':' + self.children[-1].get_operators_to_root()

        # Combined all the data and return as a string
        return "N" + str(self.identifier) + ':' \
               + self.get_operators_to_root() + '\t' \
               + str(self.order_of_expansion) + ' ' \
               + str(self.cost) + ' ' \
               + str(self.heuristic) + ' ' \
               + str(self.f) + '\n' \
               + 'Children:\t{' + children_str + '}'

########################################################################################################################
########################################################################################################################

class SearchGraph:
    '''
    This class is used to maintain all the necessary information for the search
    '''
    def __init__(self, size):
        '''
        Instantiate the class with the default values and gets the size as an input
        @param size: the height and width of the map (size x size)
        '''
        self.OPEN = []                                                              # Frontier list
        self.CLOSED = []                                                            # Explored list
        self.ACTIONS = [['LU','U','RU'],['L','O','R'],['LD','D','RD']]              # All Actions
        self.MAP = [[0 for h in range(int(size))] for w in range(int(size))]        # Real map
        self.NODEMAP = [[None for h in range(int(size))] for w in range(int(size))] # Node map
        self.GOAL_COORD = [0,0]                                                     # Coordinates of the goal node
        self.node_count = 0                                                         # Number of nodes generated
        self.expansion_count = 1                                                    # Number of nodes expanded

        self.options = {                            # set of options for the graph search
            'display_output' : True,                # flag to decide whether to display the output or not
            'display_map' : True,                   # flag to decide whether to display the map in the output
            'display_node_expansion' : 0,           # integer to keep track of the count of node expansions to print
            'goal_reached': False,                  # flag to keep track whether the goal has been reached
            'algorithm' : 'D',                      # stores which algorithm to use for the search
            'bound' : 10,                           # stores the bound for the DLS search
            'show_time' : True                      # flag to store whether to display time taken
        }

    def add_map(self, map):
        '''
        add the input map into the graph search instance, generate the inital start node and determine the coordinates
        of the goal node
        @param map: the input map after reading the file
        @return: none
        '''
        for i in range(len(self.MAP)):
            for j in range(len(self.MAP)):
                self.MAP[i][j] = map[i + 1][j]                                      # add the content to the map
                if map[i+1][j] == 'S':
                    new_Node = Node(self.node_count, 'S', 1, 0, 0, None, [i, j])    # generate the start node
                    self.NODEMAP[i][j] = new_Node                                   # add the start node to the node map
                    self.OPEN.append(new_Node)                                      # add start node to open list
                    self.node_count += 1                                            # increase node count
                if map[i + 1][j] == 'G':
                    self.GOAL_COORD[0] = i + 1                                      # store the goal coordinates
                    self.GOAL_COORD[1] = j

    def expand(self, node):
        '''

        @param node:
        @return:
        '''
        for i in range(-1,2):
            for j in range(-1,2):
                if i != 0 or j != 0:
                    new_X = node.coordinates[0] + i
                    new_Y = node.coordinates[1] + j
                    diagonal = self.check_diagonal(i,j)
                    # Case 1 (Out of Bounds)
                    if new_X >= 0 and new_Y >= 0 and new_X < len(self.MAP) and new_Y < len(self.MAP):
                        # Case 2 (Ridge)
                        if not self.check_ridge(node.coordinates[0], node.coordinates[1], new_X,new_Y,diagonal):
                            new_Cost = self.get_cost(diagonal)
                            # Case 3 (Check if its in the ancestor list)
                            if self.NODEMAP[new_X][new_Y] not in self.CLOSED:
                                if self.NODEMAP[new_X][new_Y] is not None:

                                    new_Node = Node(self.NODEMAP[new_X][new_Y].identifier,
                                                    self.ACTIONS[i+1][j+1],
                                                    self.expansion_count,
                                                    node.cost + new_Cost,
                                                    self.heuristic(new_X,new_Y),
                                                    node,
                                                    [new_X, new_Y]
                                                    )

                                    if self.check_goal(new_X,new_Y):
                                        new_Node.isGOAL = True
                                    if new_Node.cost < self.NODEMAP[new_X][new_Y].cost :
                                        self.NODEMAP[new_X][new_Y] = new_Node

                                else:
                                    new_Node = Node(self.node_count,
                                                    self.ACTIONS[i + 1][j + 1],
                                                    self.expansion_count,
                                                    node.cost + new_Cost,
                                                    self.heuristic(new_X, new_Y),
                                                    node,
                                                    [new_X, new_Y]
                                                    )

                                    if self.check_goal(new_X, new_Y):
                                        new_Node.isGOAL = True

                                    self.NODEMAP[new_X][new_Y] = new_Node
                                    self.node_count += 1
                                    self.OPEN.append(self.NODEMAP[new_X][new_Y])

                                node.children.append(self.NODEMAP[new_X][new_Y])



    def check_diagonal(self,i,j):
        '''
        determine whether the move is a diagonal using the increments
        @param i: change in the x direction
        @param j: change in the y direction
        @return: if diagonal move then return True
        '''
        if (i == -1 and j == -1) or (i == -1 and j == 1) or (i == 1 and j == -1) or (i == 1 and j == 1):
            return True
        return False

    def get_cost(self,diagonal):
        '''
        get the cost of the move, if the move is diagonal then the cost will be 1 else cost will be 2
        @param diagonal: flag to determine whether the move is diagonal
        @return:
        '''
        if diagonal:
            return 1
        else:
            return 2

    def check_ridge(self,current_X, current_Y, new_X,new_Y, diagonal):
        if self.MAP[new_X][new_Y] == 'X':
            return True
        if diagonal and new_X - 1 >= 0 and (new_X - 1 != current_X - 2) and self.MAP[new_X - 1][new_Y] == 'X':
            return True
        if diagonal and new_X + 1 < len(self.MAP) and (new_X + 1 != current_X + 2) and self.MAP[new_X + 1][new_Y] == 'X':
            return True
        if diagonal and new_Y - 1 >= 0 and (new_Y - 1 != current_Y - 2) and self.MAP[new_X][new_Y - 1] == 'X':
            return True
        if diagonal and new_Y + 1 < len(self.MAP) and (new_Y + 1 != current_Y + 2) and self.MAP[new_X][new_Y + 1] == 'X':
            return True
        return False

    def check_goal(self,new_X,new_Y):
        if self.MAP[new_X][new_Y] == 'G':
            return True
        return False

    def heuristic(self,x,y):
        dx = abs(x - self.GOAL_COORD[0])
        dy = abs(y - self.GOAL_COORD[1])

        return max(dx,dy)

    def order(self):
        self.OPEN.sort(key=lambda x: x, reverse=True)


    def search(self):
        start = time.time()
        while len(self.OPEN) > 0 :

            current_node = self.OPEN.pop()

            if self.options['algorithm'] == 'D' and len(current_node) > self.options['bound']:
                continue

            generated_solution = current_node.get_operators_to_root()

            self.CLOSED.append(current_node)

            self.expand(current_node)
            current_node.order_of_expansion = self.expansion_count
            self.expansion_count += 1

            if self.options['algorithm'] == 'A':
                self.order()

            self.display(current_node)

            if current_node.isGOAL:
                self.options['goal_reached'] = True
                if self.options['display_output']:
                    self.display(current_node)
                    if self.options['show_time']:
                        end = time.time()
                        print("Time Taken for " + self.options['algorithm'] + ": " + str(end - start))
                return generated_solution

        if self.options['display_output'] :
            print("NO-PATH")
            if self.options['show_time']:
                end = time.time()
                print("Time Taken for " + self.options['algorithm'] + ": " + str(end - start))

        return "NO-PATH"


    def get_open_list_as_string(self):
        output = ''
        for open_node in self.OPEN:
            output += '(N' + str(open_node.identifier) + ':' + open_node.get_operators_to_root() \
                      + ' ' + str(open_node.cost) + ' ' + str(round(open_node.heuristic,2)) + ' ' + str(round(open_node.f,2)) +  ')'
        return output

    def get_closed_list_as_string(self):
        output = ''
        for closed_node in self.CLOSED:
            output += '(N' + str(closed_node.identifier) + ':' + closed_node.get_operators_to_root() + ' '\
                      + str(closed_node.order_of_expansion)\
                      + ' ' + str(closed_node.cost) + ' ' + str(round(closed_node.heuristic,2)) + ' ' + str(round(closed_node.f,2)) +  ')'
        return output


    def display(self,current_node):
        output = ''

        if self.options['display_node_expansion'] > 0:
            output += str(current_node) + '\n'

            output += 'OPEN:\t{'
            output += self.get_open_list_as_string()
            output += '}\n'

            output += 'CLOSED:\t{'
            output += self.get_closed_list_as_string()
            output += '}\n'

            self.options['display_node_expansion'] -= 1
            print(output)

        if self.options['goal_reached'] :
            while current_node != None:
                map = ''
                if self.options['display_map']:
                    for i in range(len(self.MAP)):
                        for j in range(len(self.MAP[0])):
                            if current_node.coordinates[0] == i and current_node.coordinates[1] == j and \
                                    self.MAP[current_node.coordinates[0]][current_node.coordinates[1]] != 'G' and \
                                    self.MAP[current_node.coordinates[0]][current_node.coordinates[1]] != 'S':
                                map += '*'
                            else:
                                map += self.MAP[i][j]
                        map += '\n'
                current_node.get_operators_to_root()
                output = map  + '\n' +  current_node.get_operators_to_root() + ' ' + str(current_node.cost) + '\n\n' + output
                current_node = current_node.parent
            print(output)

def graphsearch(map, flag, procedure_name):

    map_size = map[0]
    search_graph = SearchGraph(map_size)
    search_graph.add_map(map)
    search_graph.options['display_node_expansion'] = flag

    if procedure_name == "D":
        bound = 300  # you have to determine its value
        # print("your code for DLS goes here")
        search_graph.options['bound'] = bound
        search_graph.options['algorithm'] = 'D'
    elif procedure_name == "A":
        # print("your code for A/A* goes here")
        search_graph.options['algorithm'] = 'A'
    else:
        print("invalid procedure name")
    solution = search_graph.search()
    return solution

def read_from_file(file_name):
    # You can change the file reading function to suit the way
    # you want to parse the file
    file_handle = open(file_name)
    map = file_handle.readlines()

    # Remove the new character lines
    for i in range(len(map)):
        map[i] = str.strip(map[i],'\n')\

    return map


###############################################################################
########### DO NOT CHANGE ANYTHING BELOW ######################################
###############################################################################

def write_to_file(file_name, solution):
    file_handle = open(file_name, 'w')
    file_handle.write(solution)

def main():
    # create a parser object
    parser = ap.ArgumentParser()

    # specify what arguments will be coming from the terminal/commandline
    parser.add_argument("input_file_name", help="specifies the name of the input file", type=str)
    parser.add_argument("output_file_name", help="specifies the name of the output file", type=str)
    parser.add_argument("flag", help="specifies the number of steps that should be printed", type=int)
    parser.add_argument("procedure_name", help="specifies the type of algorithm to be applied, can be D, A", type=str)


    # get all the arguments
    arguments = parser.parse_args()

##############################################################################
# these print statements are here to check if the arguments are correct.
#    print("The input_file_name is " + arguments.input_file_name)
#    print("The output_file_name is " + arguments.output_file_name)
#    print("The flag is " + str(arguments.flag))
#    print("The procedure_name is " + arguments.procedure_name)
##############################################################################

    # Extract the required arguments

    operating_system = platform.system()

    if operating_system == "Windows":
        input_file_name = arguments.input_file_name
        input_tokens = input_file_name.split("\\")
        if not re.match(r"(INPUT\\input)(\d)(.txt)", input_file_name):
            print("Error: input path should be of the format INPUT\input#.txt")
            return -1

        output_file_name = arguments.output_file_name
        output_tokens = output_file_name.split("\\")
        if not re.match(r"(OUTPUT\\output)(\d)(.txt)", output_file_name):
            print("Error: output path should be of the format OUTPUT\output#.txt")
            return -1
    else:
        input_file_name = arguments.input_file_name
        input_tokens = input_file_name.split("/")
        if not re.match(r"(INPUT/input)(\d)(.txt)", input_file_name):
            print("Error: input path should be of the format INPUT/input#.txt")
            return -1

        output_file_name = arguments.output_file_name
        output_tokens = output_file_name.split("/")
        if not re.match(r"(OUTPUT/output)(\d)(.txt)", output_file_name):
            print("Error: output path should be of the format OUTPUT/output#.txt")
            return -1

    flag = arguments.flag
    procedure_name = arguments.procedure_name


    try:
        map = read_from_file(input_file_name) # get the map
    except FileNotFoundError:
        print("input file is not present")
        return -1
    # print(map)
    solution_string = "" # contains solution
    write_flag = 0 # to control access to output file

    # take a decision based upon the procedure name
    if procedure_name == "D" or procedure_name == "A":
        solution_string = graphsearch(map, flag, procedure_name)
        write_flag = 1
    else:
        print("invalid procedure name")

    # call function write to file only in case we have a solution
    if write_flag == 1:
        write_to_file(output_file_name, solution_string)

if __name__ == "__main__":
    # graphsearch(read_from_file('INPUT/input6.txt'), 0, "A")
    main()