from math import sqrt
import numpy as np
import vtk  
import random



dt = 0.001

class Cube:
    def __init__(self, center, offset): 
        self.center= center
        self.x_min = center[0] - offset
        self.x_max = center[0] + offset
        self.z_min = center[1] - offset
        self.z_max = center[1] + offset
        self.y_min = -10
        self.y_max = -10+2*offset
        self.planes = [
            Plane([1.0, 0.0,0.0], self.x_max),
            Plane([-1.0,0.0,0.0], -self.x_min),
            Plane([0.0,0.0, 1.0], self.z_max),
            Plane([0.0,0.0,-1.0], -self.z_min),
            Plane([0.0,1.0, 0.0], self.y_max),
            Plane([ 0.0,-1.0, 0.0], -self.y_min)
        ]

class Sphere:
    def __init__(self):
        self.G = np.array([0.0, -9.81, 0.0], dtype=np.float64)
        self.Po = np.array([0.0, 0.0, 0.0], dtype=np.float64)
        self.Vo = np.array([0.0, 0.0, 0.0], dtype=np.float64)
        self.r = 0.0 
        self.m = 0.3
        self.F = np.zeros(3, dtype=np.float64) 

        self.collision = False
        self.collision_cube=False
        self.collision_force = np.zeros(3, dtype=np.float64)
        self.collision_cube_force = np.zeros(3, dtype=np.float64)

    def force(self, v):
        self.F = self.m * self.G 
        if self.collision:
            self.F += self.collision_force
        if self.collision_cube:
            self.F += self.collision_cube_force    

    def acceleration(self, p, v):
        self.force(v)
        return self.F / self.m
    
    def euler2(self, dt):
        self.Vo += self.acceleration(self.Po, self.Vo) * dt
        p_new = self.Po + self.Vo * dt
        self.Po = p_new
    
class Plane:
    def __init__(self,normal_vector:list[float],offset:float):
        self.n = np.array(normal_vector, dtype=np.float64)
        self.offset = offset



n_spheres = 7

L = [Sphere() for i in range(n_spheres)]
K=[]
planes = [
    Plane([1.0, 0.0,0.0], -10.0),
    Plane([-1.0,0.0,0.0], -10.0),
    Plane([0.0, 1.0,0.0], -10.0),
    Plane([0.0,-1.0,0.0], -10.0),
    Plane([0.0,0.0, 1.0], -10.0),
    Plane([0.0,0.0,-1.0], -10.0)
]



def distance(point_a, point_b):
    return sqrt((point_b[0]-point_a[0])**2 + (point_b[1]-point_a[1])**2)

def generate_cube(offset):
    n_attempts = 1000
    for i in range(n_attempts):
        x, z = random.randrange(-8,8), random.randrange(-8,8)

        intersection = False
        for cube in K:
            if distance((x, z), cube.center) <= 2*offset*sqrt(2):
                intersection = True
        for sphere in L:
            if distance((x, z), (sphere.Po[0], sphere.Po[2])) <= offset*sqrt(2)+sphere.r:
                intersection = True
        if intersection == True:
            continue
        else:
            return Cube((x,z), offset)
    raise Exception(f"Number of attempts {n_attempts} exceeded")

def generate_cubes(n_cubes, cube_size):
    for i in range(n_cubes):
        cube = generate_cube(cube_size/2)
        K.append(cube)        


def define_objects():
    L[0].Vo = np.array([-40.0, 0.0, 0], dtype=np.float64)
    L[0].Po = np.array([7.0, -9.6, 0.0], dtype=np.float64)
    L[0].r = 0.3
    L[0].m = 0.6

    L[1].Vo = np.array([0, 0, 0], dtype=np.float64)
    L[1].Po = np.array([4.5, -9.6, 0.0], dtype=np.float64)
    L[1].r = 0.3
    L[1].m = 0.2

    L[2].Vo = np.array([0, 0, 0], dtype=np.float64)
    L[2].Po = np.array([1.0, -9.6, -0.5], dtype=np.float64)
    L[2].r = 0.3
    L[2].m = 0.2

    L[3].Vo = np.array([0, 0, 0], dtype=np.float64)
    L[3].Po = np.array([1.0, -9.6, 0.5], dtype=np.float64)
    L[3].r = 0.3
    L[3].m = 0.2

    L[4].Vo = np.array([0, 0, 0], dtype=np.float64)
    L[4].Po = np.array([-1.5, -9.6, 0.0], dtype=np.float64)
    L[4].r = 0.3
    L[4].m = 0.2

    L[5].Vo = np.array([0, 0, 0], dtype=np.float64)
    L[5].Po = np.array([1.0, -9.6, 1.5], dtype=np.float64)
    L[5].r = 0.3
    L[5].m = 0.2

    L[6].Vo = np.array([0, 0, 0], dtype=np.float64)
    L[6].Po = np.array([1.0, -9.6, -1.5], dtype=np.float64)
    L[6].r = 0.3
    L[6].m = 0.2

    generate_cubes(3, 3)

define_objects()

def check_collision_with_room():
    for sphere in L:
        sphere.collision = False 
        sphere.collision_force = np.zeros(3, dtype=np.float64) 

        for plane in planes: 
            if np.dot(sphere.Po, plane.n) <= sphere.r + plane.offset:
                    n = plane.n / np.linalg.norm(plane.n)
                    vr = sphere.Vo
                    vrn = np.dot(vr, n)
                    if vrn < 0.0:
                        J = (vrn * (0.4 + 1) * sphere.m)
                        Fi = -n * (J / dt)
                        sphere.collision_force += Fi  
                        sphere.collision = True 

def check_collision_with_cubes():
    for sphere in L:
        sphere.collision_cube = False
        sphere.collision_cube_force = np.zeros(3, dtype=np.float64)

        for cube in K:
            
            Kmin = np.array([cube.x_min, cube.y_min, cube.z_min], dtype=np.float64)
            Kmax = np.array([cube.x_max, cube.y_max, cube.z_max], dtype=np.float64)

            
            q = np.minimum(np.maximum(sphere.Po, Kmin), Kmax)

            
            diff = sphere.Po - q

            if np.dot(diff, diff) <= sphere.r*sphere.r:

                eps = 1e-9
                if np.linalg.norm(diff) > eps:
                    n = diff / np.linalg.norm(diff)
                else:
                    n = diff/eps 
                vr = sphere.Vo 
                vrn = np.dot(vr, n) 
                
                if vrn < 0.0: 
                    J = -vrn * (0.4 + 1) * sphere.m 
                    Fi = n * (J / dt) 

                    sphere.collision_cube_force += Fi 
                    sphere.collision_cube = True 

                break


def sphere_collisions(sphere_a, sphere_b):
    distance = np.linalg.norm(sphere_a.Po - sphere_b.Po)
    
    if distance < (sphere_a.r + sphere_b.r):
        normal = (sphere_b.Po - sphere_a.Po) / distance

        if np.array_equal(sphere_b.Vo, np.zeros(3, dtype=np.float64)):
            v1 = sphere_a.Vo
            v1n = np.dot(v1, normal)
            if v1n > 0: 
                v1n_new = -v1n 
                sphere_a.Vo = v1 - v1n * normal + v1n_new * normal
        else: 
            v1 = sphere_a.Vo
            v2 = sphere_b.Vo
            
            v1n = np.dot(v1, normal)
            v2n = np.dot(v2, normal)
            
            m1, m2 = sphere_a.m, sphere_b.m
            v1n_new = (v1n * (m1 - m2) + 2 * m2 * v2n) / (m1 + m2)
            v2n_new = (v2n * (m2 - m1) + 2 * m1 * v1n) / (m1 + m2)
            
            sphere_a.Vo = v1 - v1n * normal + v1n_new * normal
            sphere_b.Vo = v2 - v2n * normal + v2n_new * normal

        overlap = (sphere_a.r + sphere_b.r) - distance-0.01
        correction = normal * overlap / 2
        sphere_a.Po += correction
        sphere_b.Po -= correction

def check_all_spheres_collisions():
    for i in range(len(L)):
        for j in range(i+1, len(L)):
            sphere_collisions(L[i], L[j])



class VtkLoopCallback:
    def __init__(self, sphere_sources, render_window):
        self.sphere_sources = sphere_sources 
        self.render_window = render_window 
        self.t = 0.0 
        self.step = 0 

    def __call__(self, caller, event):
        key = caller.GetKeySym() 
        if key == "k": 
            while self.t <= 10:
                self.t += dt 

                check_all_spheres_collisions()
                check_collision_with_cubes()
                check_collision_with_room()

                for i, sphere in enumerate(L): 
                    sphere.euler2(dt) 
                    if i == 4: 
                        sphere.Vo = np.zeros(3, dtype=np.float64)
                    self.sphere_sources[i].SetCenter(sphere.Po[0], sphere.Po[1] , sphere.Po[2]) 

                self.step += 1 
                if self.step == 5: 
                    self.render_window.Render()
                    self.step = 0

def main():
    sphere_sources = []
    sphere_actors = []

    cube_sources=[]
    cube_actors=[]
    

    for sphere_data in L:
        sphere = vtk.vtkSphereSource() 
        sphere.SetCenter(sphere_data.Po[0], sphere_data.Po[1], sphere_data.Po[2])
        sphere.SetRadius(sphere_data.r)
        sphere.SetPhiResolution(20)  
        sphere.SetThetaResolution(20)

        mapper = vtk.vtkPolyDataMapper()
        mapper.SetInputConnection(sphere.GetOutputPort())  

        actor = vtk.vtkActor() 
        actor.SetMapper(mapper)  
        actor.GetProperty().SetColor(0, 1, 0) 

        sphere_sources.append(sphere) 
        sphere_actors.append(actor)

    for cube_data in K:
        cube1 = vtk.vtkCubeSource() 
        cube1.SetBounds(cube_data.x_min, cube_data.x_max, cube_data.y_min, cube_data.y_max, cube_data.z_min, cube_data.z_max)


        mapper = vtk.vtkPolyDataMapper()
        mapper.SetInputConnection(cube1.GetOutputPort())  

        actor = vtk.vtkActor() 
        actor.SetMapper(mapper)  
        actor.GetProperty().SetColor(0.7, 0, 0.3) 

        cube_sources.append(cube1)
        cube_actors.append(actor)

    room_source = vtk.vtkCubeSource() 
    room_source.SetBounds( -10.0, 10.0, -10.0, 10.0, -10.0, 10.0)

    room_mapper = vtk.vtkPolyDataMapper()  
    room_mapper.SetInputConnection(room_source.GetOutputPort())  

    room_actor = vtk.vtkActor() 
    room_actor.SetMapper(room_mapper) 
    room_actor.GetProperty().SetOpacity(0.2)
    
    cube_mapper1 = vtk.vtkPolyDataMapper()  
    cube_mapper1.SetInputConnection(cube1.GetOutputPort())  

    cube_actor1 = vtk.vtkActor()
    cube_actor1.SetMapper(cube_mapper1) 
    cube_actor1.GetProperty().SetColor(1.0,0.0,0.0)


    renderer = vtk.vtkRenderer()  
    render_window = vtk.vtkRenderWindow() 
    render_window.AddRenderer(renderer) 
    render_window.SetSize(1000, 800)  

    renderer.AddActor(room_actor)
    renderer.AddActor(cube_actor1)
    renderer.AddActor(sphere_actors[0]) 


    for actor in sphere_actors:  
         renderer.AddActor(actor)

    sphere_actors[0].GetProperty().SetColor(1, 0, 0)

    for actor in cube_actors:  
        renderer.AddActor(actor)

    interactor = vtk.vtkRenderWindowInteractor()  
    interactor.SetRenderWindow(render_window) 

    callback = VtkLoopCallback(sphere_sources, render_window)  
    interactor.AddObserver(vtk.vtkCommand.KeyPressEvent, callback) 

    render_window.Render()  
    interactor.Initialize() 
    interactor.Start()  

if __name__ == "__main__":
    main()