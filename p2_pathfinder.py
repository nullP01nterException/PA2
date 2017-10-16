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
    tent={}
    visited=set()
    distances={}
    
    print("source", source_point)
    print("dest", destination_point)
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

    print(detail_points)
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
