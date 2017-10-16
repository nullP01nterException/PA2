from math import inf, sqrt
from heapq import heappop, heappush

def point_distance(p1, p2):
    dist = sqrt((p1[0]-p2[0])**2 + (p1[1]-p2[1])**2)
    return dist

def find_path (source_point, destination_point, mesh):

    """
    Searches for a path from source_point to destination_point through the mesh

    Args:
        source_point: starting point of the pathfinder
        destination_point: the ultimate goal the pathfinder must reach
        mesh: pathway constraints the path adheres to

    Returns:

        A path (list of points) from source_point to destination_point if exists
        A list of boxes explored by the algorithm
    """
    path = []
    box_path = []
    boxes = {}
    queue=[]
    parent={}
    detail_points={}
    visited=set()
    
    print("source", source_point)
    print("dest", destination_point)
    for box in mesh['boxes']:
        if(source_point[1]>box[2] and source_point[1]<box[3] and source_point[0]>box[0] and source_point[0]<box[1]):
            boxes[box]='source'
            source_box=box
            queue.append(box)
            parent[box]=source_box
        if(destination_point[1]>box[2] and destination_point[1]<box[3] and destination_point[0]>box[0] and destination_point[0]<box[1]):
            boxes[box]='destination'
            destination_box=box

    while queue:
        bfspath=queue.pop(0)
        if bfspath == destination_box:
            temp=bfspath
            while parent[temp] != source_box or temp != source_box:
                box_path.append(temp)
                temp=parent[temp]
                print(temp)
            box_path.append(source_box)
            break
        for adjacent in mesh.get('adj', {}).get(bfspath):
            if adjacent not in visited:
                 parent[adjacent]=bfspath
                 if adjacent not in queue:
                     queue.append(adjacent)
                 visited.add(bfspath)
    box_path.reverse()
    for index in box_path:
        boxes[index]=0
    detail_points[source_box]=source_point
    #detail_points[destination_box]=destination_point

    for index in range(len(box_path)):
        if(index==0):
            continue
        
        if box_path[index] not in detail_points:
            start_point=detail_points[box_path[index-1]]
            x_range_min=max(box_path[index][0],box_path[index-1][0])
            x_range_max=min(box_path[index][1],box_path[index-1][1])
            y_range_min=max(box_path[index][2],box_path[index-1][2])
            y_range_max=min(box_path[index][3],box_path[index-1][3])
            if x_range_min == x_range_max:
                if start_point[1] > y_range_min and start_point[1] < y_range_max:
                    detail_points[box_path[index]]=(x_range_min, start_point[1])
                elif point_distance(start_point, (x_range_min, y_range_min)) < point_distance(start_point, (x_range_min, y_range_max)):
                    detail_points[box_path[index]]=(x_range_min, y_range_min)
                else:
                    detail_points[box_path[index]]=(x_range_min, y_range_max)
            if y_range_min == y_range_max:
                if start_point[0] > x_range_min and start_point[0] < x_range_max:
                    detail_points[box_path[index]]=(start_point[0], y_range_min)
                elif point_distance(start_point, (x_range_min, y_range_min)) < point_distance(start_point, (x_range_max, y_range_min)):
                    detail_points[box_path[index]]=(x_range_min, y_range_min)
                else:
                    detail_points[box_path[index]]=(x_range_max, y_range_min)

    for box_path_index in range(len(box_path)-1):
        curr_box_point = box_path[box_path_index]
        next_box_point = box_path[box_path_index+1]
        segment = (detail_points[curr_box_point], detail_points[next_box_point])
        path.append(segment)

    if len(box_path) == 0:
        print("No path found!")
    detail_points[destination_box]=destination_point
    return path, boxes.keys()
