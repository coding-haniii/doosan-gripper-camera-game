"""Find path in previously unseen black-wall/white-corridor top-view maze.
Explicit start/end pixels required (or sample metadata). No claim of universal maze understanding.
"""
from collections import deque
import cv2
import numpy as np

def solve_maze(image, start, goal, threshold=170, clearance=2, stride=8):
    gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY) if image.ndim == 3 else image
    walk = (gray > threshold).astype(np.uint8)
    if clearance:
        walk = cv2.erode(walk, np.ones((clearance*2+1,clearance*2+1), np.uint8))
    h,w = walk.shape
    def nearest(p):
        x,y = map(int,p)
        if not (0 <= x < w and 0 <= y < h): raise ValueError('Point out of image')
        if walk[y,x]: return (x,y)
        for radius in range(1, max(10,clearance+6)):
            for dy in range(-radius,radius+1):
                for dx in range(-radius,radius+1):
                    xx,yy=x+dx,y+dy
                    if 0<=xx<w and 0<=yy<h and walk[yy,xx]: return xx,yy
        raise ValueError(f'Start or goal on wall: {p}')
    start,goal=nearest(start),nearest(goal)
    q=deque([start]); parent={start:None}
    while q:
        x,y=q.popleft()
        if (x,y)==goal: break
        for dx,dy in ((0,1),(1,0),(0,-1),(-1,0)):
            p=(x+dx,y+dy)
            if 0<=p[0]<w and 0<=p[1]<h and walk[p[1],p[0]] and p not in parent:
                parent[p]=(x,y); q.append(p)
    if goal not in parent: raise ValueError('No collision-free path found')
    path=[]; node=goal
    while node is not None: path.append(node); node=parent[node]
    path.reverse()
    # Keep bends even if sampling interval would skip them.
    chosen=[path[0]]
    for i in range(1,len(path)-1):
        p0,p1,p2=path[i-1:i+2]
        if (p1[0]-p0[0],p1[1]-p0[1]) != (p2[0]-p1[0],p2[1]-p1[1]) or i%max(1,stride)==0:
            chosen.append(p1)
    if chosen[-1]!=path[-1]:chosen.append(path[-1])
    return chosen

def overlay(image, path):
    colored = cv2.cvtColor(image,cv2.COLOR_GRAY2BGR) if image.ndim==2 else image.copy()
    for a,b in zip(path,path[1:]): cv2.line(colored,a,b,(0,0,255),2)
    cv2.circle(colored,path[0],6,(0,255,0),-1)
    cv2.circle(colored,path[-1],6,(255,0,0),-1)
    return colored

def generate_maze(seed=7, cells=8, scale=24):
    rng=np.random.default_rng(seed)
    walls=np.ones((2*cells+1,2*cells+1),np.uint8)
    visited={(0,0)}; stack=[(0,0)]; walls[1,1]=0
    while stack:
        r,c=stack[-1]
        neighbors=[(r+dr,c+dc) for dr,dc in ((1,0),(-1,0),(0,1),(0,-1))
                   if 0<=r+dr<cells and 0<=c+dc<cells and (r+dr,c+dc) not in visited]
        if not neighbors: stack.pop(); continue
        nr,nc=neighbors[int(rng.integers(len(neighbors)))]
        walls[r+nr+1,c+nc+1]=0; walls[2*nr+1,2*nc+1]=0
        visited.add((nr,nc)); stack.append((nr,nc))
    img=np.where(walls==0,255,0).astype(np.uint8)
    img=cv2.resize(img,None,fx=scale,fy=scale,interpolation=cv2.INTER_NEAREST)
    start=((1*scale)+scale//2,(1*scale)+scale//2)
    goal=((2*cells-1)*scale+scale//2,(2*cells-1)*scale+scale//2)
    return img,start,goal
