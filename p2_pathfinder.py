from math import inf, sqrt
from heapq import heappop, heappush
#Ruihong Yu
#ruyu
#PA2
#Collaborated with Jolina Lam

#Helper function to calculate distance between two points
def point_distance(p1, p2):
    dist = sqrt((p1[0]-p2[0])**2 + (p1[1]-p2[1])**2)
    return dist

#This is normal A* please don't use
def bad_find_path (source_point, destination_point, mesh):

    path = []
    box_path = []
    boxes = {}
    queue=[]
    parent={}
    detail_points={}
    tent={}
    visited=set()
    distances={}
    
    for box in mesh['boxes']:
        if(source_point[1]>box[2] and source_point[1]<box[3] and source_point[0]>box[0] and source_point[0]<box[1]):
            boxes[box]='source'
            source_box=box
            queue.append((0,box))
            distances[box]=0
            parent[box]=source_box
        if(destination_point[1]>box[2] and destination_point[1]<box[3] and destination_point[0]>box[0] and destination_point[0]<box[1]):
            boxes[box]='destination'
            destination_box=box
    detail_points[source_box]=source_point
    detail_points[destination_box]=destination_point
    while queue:
        info = heappop(queue)
        curr_dist = distances[info[1]]
        curr_box = info[1]
        
        if curr_box == destination_box:
            walker=curr_box
            while walker != source_box:
                box_path.append(walker)
                walker=parent[walker]
            box_path.append(source_box)
            break
        for adjacent in mesh.get('adj', {}).get(curr_box):

            start_point=detail_points[curr_box]
            x_range_min=max(curr_box[0],adjacent[0])
            x_range_max=min(curr_box[1],adjacent[1])
            y_range_min=max(curr_box[2],adjacent[2])
            y_range_max=min(curr_box[3],adjacent[3])
            if x_range_min == x_range_max:
                if start_point[1] >= y_range_min and start_point[1] < y_range_max:
                    tent[adjacent]=(x_range_min, start_point[1])
                elif point_distance(start_point, (x_range_min, y_range_min)) < point_distance(start_point, (x_range_min, y_range_max)):
                    tent[adjacent]=(x_range_min, y_range_min)
                else:
                    tent[adjacent]=(x_range_min, y_range_max)
            if y_range_min == y_range_max:
                if start_point[0] >= x_range_min and start_point[0] < x_range_max:
                    tent[adjacent]=(start_point[0], y_range_min)
                elif point_distance(start_point, (x_range_min, y_range_min)) < point_distance(start_point, (x_range_max, y_range_min)):
                    tent[adjacent]=(x_range_min, y_range_min)
                else:
                    tent[adjacent]=(x_range_max, y_range_min)

            pathcost=curr_dist+point_distance(detail_points[curr_box], tent[adjacent])
            if adjacent not in distances or pathcost < distances[adjacent]:
                detail_points[adjacent]=tent[adjacent]
                distances[adjacent] = pathcost
                parent[adjacent]= curr_box
                adjusted_cost = pathcost + point_distance(detail_points[curr_box], destination_point)
                heappush(queue, (adjusted_cost, adjacent))
    if len(box_path) == 0:
        print("No path found!")    
    else:
        for box_path_index in range(len(box_path)-1):
            segment = (detail_points[box_path[box_path_index]],detail_points[box_path[box_path_index+1]])
            path.append(segment)
    if len(box_path) == 1:
        path.append((source_point, destination_point))
    elif len(box_path) > 1:
        path.append((detail_points[box_path[0]], destination_point))
    detail_points[destination_box]=destination_point
    for box in box_path:
        boxes[box]=0
    return path, boxes.keys()

#Use this please
def find_path(source_point, destination_point, mesh):
    path = []
    box_path = []
    boxes = {}
    queue = []
    forward_parent = {}
    backward_parent = {}
    forward_detail_points = {}
    backward_detail_points={}
    tent = {}
    visited = set()
    forward_distances = {}
    backward_distances ={}

    for box in mesh['boxes']:
        if (source_point[1] > box[2] and source_point[1] < box[3] and source_point[0] > box[0] and source_point[0] <
            box[1]):
            boxes[box] = 'source'
            source_box = box
            forward_distances[box] = 0
            forward_parent[box] = source_box
        if (destination_point[1] > box[2] and destination_point[1] < box[3] and destination_point[0] > box[0] and
                    destination_point[0] < box[1]):
            boxes[box] = 'destination'
            destination_box = box
            backward_parent[box] = box
            backward_distances[box] = 0
    forward_detail_points[source_box] = source_point
    backward_detail_points[destination_box] = destination_point
    heappush(queue,(0, source_box, 'destination'))
    heappush(queue,(0, destination_box, 'source'))
    
    #Bidirectional A*
    while queue:
        info = heappop(queue)
        curr_box = info[1]
        if info[2] == "destination":
            curr_dist = forward_distances[info[1]] #break when current box is in parent dictionary
            if curr_box in backward_parent:
                walker = curr_box
                temp = curr_box #put first half of boxes in box path
                while walker != source_box:
                    box_path.append(walker)
                    walker = forward_parent[walker]
                box_path.append(source_box)
                walker = backward_parent[temp] #put second half of boxes in box path
                while walker != destination_box:
                    box_path.insert(0,walker)
                    walker = backward_parent[walker]
                box_path.insert(0,destination_box)
                break
        else:
            curr_dist = backward_distances[info[1]] #break when current box is in parent dictionary
            if curr_box in forward_parent:
                walker = curr_box
                temp = curr_box #put first half of boxes in box path
                while walker != destination_box:
                    box_path.append(walker)
                    walker = backward_parent[walker]
                box_path.append(destination_box)
                walker = forward_parent[temp] # put second half of boxes in box path
                while walker != source_box:
                    box_path.insert(0,walker)
                    walker = forward_parent[walker]
                box_path.insert(0,source_box)
                break

        for adjacent in mesh.get('adj', {}).get(curr_box): #calculates detail points
            if info[2]=="destination":
                start_point = forward_detail_points[curr_box]
            else:
                start_point = backward_detail_points[curr_box]
            x_range_min = max(curr_box[0], adjacent[0])
            x_range_max = min(curr_box[1], adjacent[1])
            y_range_min = max(curr_box[2], adjacent[2])
            y_range_max = min(curr_box[3], adjacent[3])
            if x_range_min == x_range_max: #checks whether the boxes intersect in a horizontal line
                if start_point[1] >= y_range_min and start_point[1] < y_range_max: #if straight line can be drawn to intersecting edge, do so
                    tent[adjacent] = (x_range_min, start_point[1])
                elif point_distance(start_point, (x_range_min, y_range_min)) < point_distance(start_point, (
                x_range_min, y_range_max)): #draws a line to the closest corner if straight line cannot be drawn
                    tent[adjacent] = (x_range_min, y_range_min)
                else:
                    tent[adjacent] = (x_range_min, y_range_max)
            if y_range_min == y_range_max: #checks whether the boxes intersect in a vertical line
                if start_point[0] >= x_range_min and start_point[0] < x_range_max: #if straight line can be drawn to intersecting edge, do so
                    tent[adjacent] = (start_point[0], y_range_min)
                elif point_distance(start_point, (x_range_min, y_range_min)) < point_distance(start_point, (
                x_range_max, y_range_min)): #draws a line to closest corner if straight line cannot be drawn
                    tent[adjacent] = (x_range_min, y_range_min)
                else:
                    tent[adjacent] = (x_range_max, y_range_min)
            if info[2]=="destination": #adjusts pathcosts accordingly with line segments
                pathcost = curr_dist + point_distance(forward_detail_points[curr_box], tent[adjacent])
            else:
                pathcost = curr_dist + point_distance(backward_detail_points[curr_box], tent[adjacent])
            if info[2] == "destination":
                if adjacent not in forward_distances or pathcost < forward_distances[adjacent]: #puts adjacent box in path if new or better path found
                    forward_detail_points[adjacent] = tent[adjacent] #updates detail point, parent, and distance
                    forward_distances[adjacent] = pathcost
                    forward_parent[adjacent] = curr_box
                    adjusted_cost = pathcost + point_distance(forward_detail_points[curr_box], destination_point)
                    heappush(queue, (adjusted_cost, adjacent, "destination")) #uses Euclidean heuristic to push to priority queue
            else:
                if adjacent not in backward_distances or pathcost < backward_distances[adjacent]: #does the same as above but for backwards search
                    backward_detail_points[adjacent] = tent[adjacent]
                    backward_distances[adjacent] = pathcost
                    backward_parent[adjacent] = curr_box
                    adjusted_cost = pathcost + point_distance(backward_detail_points[curr_box], source_point)
                    heappush(queue, (adjusted_cost, adjacent, "source"))
    #end A*
                    
    if len(box_path) == 0: #if no path
        print("No path found!")
    else:
        for box_path_index in range(len(box_path) - 1): #cycles all box paths
            if box_path_index + 1 == len(box_path): #checks for intersection
                segment = (forward_detail_points[box_path[box_path_index+1]], backward_detail_points[box_path[box_path_index+1]])
            if box_path[box_path_index] in forward_detail_points and box_path[box_path_index +1] in forward_detail_points: #adds detail points to path from respective detail point dictionaries
                segment = (forward_detail_points[box_path[box_path_index]], forward_detail_points[box_path[box_path_index+1]])
            elif box_path[box_path_index] in backward_detail_points and box_path[box_path_index +1] in backward_detail_points:
                segment = (backward_detail_points[box_path[box_path_index]], backward_detail_points[box_path[box_path_index+1]])
            path.append(segment)
        for samebox in forward_detail_points: #fixes the one line segment that doesn't connect for box with two detail points
            if samebox in backward_detail_points and forward_detail_points[samebox] != backward_detail_points[samebox] and samebox in box_path:
                path.append((backward_detail_points[samebox], forward_detail_points[samebox]))
                break
    if len(box_path) == 2 and box_path[0]==box_path[1]: #if same box
        path.append((source_point, destination_point))
    for box in box_path: #updates boxes to contain box path
        boxes[box] = 0
    return path, boxes.keys()
