"""Deterministic static collision map for the axis-aligned single-room world.

Generated files are runtime artifacts, not observations or localization truth.
Unsupported static geometry is rejected instead of silently approximated.
"""
from collections import deque
from dataclasses import dataclass
import math
import xml.etree.ElementTree as ET

OCCUPIED, UNKNOWN, FREE = 0, 128, 254


def numbers(text, count):
    values=tuple(float(x) for x in text.split())
    if len(values)!=count or not all(math.isfinite(x) for x in values):
        raise ValueError('Expected finite coordinate tuple')
    return values


def translation(element):
    pose=element.find('pose')
    if pose is None:return (0.,0.,0.)
    if pose.attrib:raise ValueError('Relative/expressed poses are not supported')
    values=numbers(pose.text or '',6)
    if any(abs(x)>1e-9 for x in values[3:]):raise ValueError('Rotated static boxes are not supported')
    return values[:3]


@dataclass(frozen=True)
class Box:
    name: str
    x: float
    y: float
    z: float
    sx: float
    sy: float
    sz: float

    def distance_xy(self,x,y):
        return math.hypot(max(abs(x-self.x)-self.sx/2,0),max(abs(y-self.y)-self.sy/2,0))


def static_boxes(world_path):
    world=ET.parse(world_path).getroot().find('world')
    if world is None or world.findall('include'):raise ValueError('Expected an explicit world without includes')
    boxes=[]
    for model in world.findall('model'):
        if model.findtext('static') not in ('true','1'):continue
        if model.findall('model') or model.findall('include'):raise ValueError('Nested static models are unsupported')
        mt=translation(model)
        for link in model.findall('link'):
            lt=translation(link)
            for collision in link.findall('collision'):
                ct=translation(collision)
                shape=collision.find('geometry')
                if shape is None or len(shape)!=1 or shape[0].tag!='box':
                    raise ValueError('Only static box collisions are supported')
                size=numbers(shape.findtext('box/size') or '',3)
                if any(v<=0 for v in size):raise ValueError('Collision dimensions must be positive')
                xyz=tuple(mt[i]+lt[i]+ct[i] for i in range(3))
                boxes.append(Box(model.attrib['name'],*xyz,*size))
    if sum(box.name=='floor' for box in boxes)!=1:raise ValueError('Expected one floor collision')
    return boxes


@dataclass
class Grid:
    resolution: float
    origin: tuple
    width: int
    height: int
    cells: list

    def cell(self,x,y):
        return math.floor((x-self.origin[0])/self.resolution),math.floor((y-self.origin[1])/self.resolution)

    def center(self,column,row):
        return (self.origin[0]+(column+.5)*self.resolution,self.origin[1]+(row+.5)*self.resolution)

    def at(self,column,row):
        if not 0<=column<self.width or not 0<=row<self.height:return UNKNOWN
        return self.cells[row*self.width+column]

    def pgm(self):
        # PGM top row is world maximum Y; internal rows start at the map origin.
        rows=[self.cells[r*self.width:(r+1)*self.width] for r in range(self.height-1,-1,-1)]
        return ('P2\n'+f'{self.width} {self.height}\n255\n'+'\n'.join(' '.join(map(str,row)) for row in rows)+'\n').encode()

    def reachable(self,start,goal,radius):
        if not math.isfinite(radius) or radius<0:raise ValueError('Invalid footprint radius')
        bound=math.ceil(radius/self.resolution)+1
        offsets=[(dx,dy) for dx in range(-bound,bound+1) for dy in range(-bound,bound+1)
                 if math.hypot(dx,dy)*self.resolution <= radius+self.resolution*math.sqrt(2)/2]
        cache={}
        def clear(cell):
            if cell not in cache:
                cache[cell]=all(self.at(cell[0]+dx,cell[1]+dy)==FREE for dx,dy in offsets)
            return cache[cell]
        first,last=self.cell(*start),self.cell(*goal)
        if not clear(first) or not clear(last):return False
        queue=deque([first]);visited={first}
        while queue:
            cell=queue.popleft()
            if cell==last:return True
            # Four-connected search prevents diagonal corner cutting.
            for dx,dy in ((1,0),(-1,0),(0,1),(0,-1)):
                nxt=(cell[0]+dx,cell[1]+dy)
                if nxt not in visited and clear(nxt):visited.add(nxt);queue.append(nxt)
        return False


def build_grid(world_path,config):
    resolution=config['resolution_m'];width=config['width_cells'];height=config['height_cells']
    origin=tuple(config['origin_xy_m']);slice_z=config['slice_height_m']
    if (not math.isfinite(resolution) or resolution<=0 or not isinstance(width,int) or not isinstance(height,int)
        or width<=0 or height<=0 or width*height>1_000_000 or len(origin)!=2
        or not all(math.isfinite(v) for v in (*origin,slice_z))):raise ValueError('Invalid map dimensions')
    boxes=static_boxes(world_path);floor=next(b for b in boxes if b.name=='floor')
    obstacles=[b for b in boxes if b.name!='floor' and abs(slice_z-b.z)<=b.sz/2]
    grid=Grid(resolution,origin,width,height,[]);half=resolution/2
    for row in range(height):
        for column in range(width):
            x,y=grid.center(column,row)
            occupied=any(abs(x-b.x)<=b.sx/2+half and abs(y-b.y)<=b.sy/2+half for b in obstacles)
            inside=abs(x-floor.x)+half<=floor.sx/2 and abs(y-floor.y)+half<=floor.sy/2
            grid.cells.append(OCCUPIED if occupied else FREE if inside else UNKNOWN)
    return grid
