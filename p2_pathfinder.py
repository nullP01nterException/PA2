from math import inf, sqrt
from heapq import heappop, heappush

def point_distance(p1, p2):
    dist = sqrt((p1[0]-p2[0])**2 + (p1[1]-p2[1])**2)
    print("dist", dist)
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
    visited=set()
    print("source", source_point)
    print("dest", destination_point)
    for box in mesh['boxes']:
        if(source_point[1]>box[2] and source_point[1]<box[3] and source_point[0]>box[0] and source_point[0]<box[1]):
            boxes[box]='source'
            source_box=box
            queue.append(box)
            parent[box]=0
        if(destination_point[1]>box[2] and destination_point[1]<box[3] and destination_point[0]>box[0] and destination_point[0]<box[1]):
            boxes[box]='destination'
            destination_box=box

    while queue:
        bfspath=queue.pop(0)
        if bfspath == destination_box:
            temp=bfspath
            while parent[temp] != source_box:
                box_path.append(temp)
                temp=parent[temp]
            box_path.append(source_box)
            break
        for adjacent in mesh.get('adj', {}).get(bfspath):
            if adjacent not in visited:
                parent[adjacent]=bfspath
                queue.append(adjacent)
                visited.add(bfspath)
    box_path.reverse()
    print("box_path", box_path)

    if len(box_path) == 0:
        print("No path found!")
    return path, boxes.keys()
