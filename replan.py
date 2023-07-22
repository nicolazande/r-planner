import os, math, random, time
from pydoc import doc
import numpy as np
import matplotlib.pyplot as plt
import matplotlib.patches as patches
from scipy.spatial.transform import Rotation as Rot
#from sqlalchemy import true
#moduli miei
import env, plotting, utils, smoothing


#nodo standard (x, y, parent)
class Node:

    def __init__(self, S):
        self.S = np.array(S) #stato
        self.parent = None


#tree: [start, goal, V(vertex set), E(edge set), QE(coda edees), QV(coda vertex), Vold(salvo per confronto)]
class Tree:

    def __init__(self, x_start, x_goal):
        self.r = 4.0 #raggio di ricerca
        self.V = set() #set di vertici connessi
        self.E = set() #set di edges connessi
        self.QE = set() #coda ordinata di edges
        self.QV = set() #coda ordinata di vertici
        self.V_old = set() #set di vertici vecchi (li uso per confronto)


#BIT* algorithm
class BITStar:

    #costruttore
    def __init__(self, x_start, x_goal, eta, p_safe):
        
        #parametri iniziali
        self.x_start = Node(x_start)
        self.x_goal = Node(x_goal)
        self.eta = eta
        self.theta = 0
        self.cMin = 0
        self.cBest = np.inf
        self.xCenter = 0
        self.C = 0
        self.m = 0
        self.replan_factor = 10
    
        #definisco environment
        self.env = env.Env()
        self.utils = utils.Utils(p_safe)
        self.x_range = self.env.x_range
        self.y_range = self.env.y_range
        self.obs_circle = self.env.obs_circle
        self.obs_rectangle = self.env.obs_rectangle
        self.obs_boundary = self.env.obs_boundary
        self.delta = self.utils.delta

        #plotting        
        self.plot_on = True
        self.animation_on = True
        if self.animation_on or self.plot_on:
            self.plotting = plotting.Plotting(x_start, x_goal)
            self.fig, self.ax = plt.subplots()

        #inizializzo albero con start e goal (lo uso sia per forward che reverse search)
        self.Tree = Tree(self.x_start, self.x_goal)

        #set e dict che uso
        self.X_unconn = set() #set di nodi non connessi ma potenzialmente migliarano soluzione
        self.g_F = dict() #dict dove salvo costo nodi

        #dati per benchmark
        self.iteration = 0
        self.benchmark_list = []


    #inizializzo nuovo batch
    def init(self):
        self.Tree.V.update([self.x_start, self.x_goal]) #aggiungo Xstart e X_goal a V(set di vertici
        self.X_unconn.add(self.x_goal) #aggiungo Xgoal a X_unconn(unconnected samples
        self.g_F[self.x_start] = 0.0 #costo vero Xstart = 0
        self.g_F[self.x_goal] = np.inf #costo vero Xgoal = inf
        #aggiorno parametri        
        self.RotationToWorldFrame()
        self.xCenter = np.array([[(self.x_start.S[0] + self.x_goal.S[0]) / 2.0],
                                [(self.x_start.S[1] + self.x_goal.S[1]) / 2.0], [0.0]])
        self.m = 300 #numero di sample per batch


    #planning
    def planning(self, steps):
        self.init()
        start_time = time.time() #inizializzo parametri di ricerca (per primo batch)
        
        #numero di batch (numero massimo di volte che eseguo resampling)
        while self.g_F[self.x_goal] >= np.inf:

            #controllo se QE e QV sono vuoti --> new batch
            if not self.Tree.QE and not self.Tree.QV:

                if self.plot_on:
                    if self.x_goal.parent is not None:
                        self.PlotFinalPath(True) #plotto path finale e interpolato con B-spline

                dm = self.Prune(self.g_F[self.x_goal]) #elimino tutti i vertici non inclusi nel nuovo ellisse
                self.X_unconn.update(self.Sample(self.m-dm)) #resample nella nuovo regione selezionata con [m] samples
                self.Tree.V_old = {v for v in self.Tree.V} #salvo V << Vold per controllo successivo
                self.Tree.QV = {v for v in self.Tree.V} #salvo QV << V --> coda di vertici da processare in ordine di costo soluzione (vincolata a passare per nodi correnti in albero)

            #-------- costruisco tree con edge in ordine di costo -----------------------------------
            #BestVertexQueueValue(QV) --> stima miglior soluzione in QV --> espando solamente finche miglioramenti possibili
            while self.Tree.QV and self.BestVertexQueueValue() <= self.BestEdgeQueueValue():
                self.ExpandVertex(self.BestInVertexQueue()) #ogni vertice selezionato --> expando edges e aggiungo a QE (con condizioni)

            if not self.Tree.QE:
                continue

            vm, xm = self.BestInEdgeQueue() #seleziono miglior edge in QE

            self.Tree.QE.remove((vm, xm)) #tolgo da QE il miglior edge (vm, xm) da processare
            #---------- check miglioramento(current_solution, current_tree) -------------------------
            if self.gF(vm) + self.calc_dist(vm, xm) + self.h_estimated(xm) < self.gF(self.x_goal): #se stima ha potenziale --> procedo (risparmio collision check molte volte)
                actual_cost = self.cost(vm, xm) # <-- costo (vm, xm) che tiene conto di collisioni
                if self.g_estimated(vm) + actual_cost + self.h_estimated(xm) < self.gF(self.x_goal): #check con costo vero
                    if self.gF(vm) + actual_cost < self.gF(xm): #controllo se anche cost_to_come(xm) migliorabile
                        
                        if xm in self.Tree.V: 
                            #xm era gia nell albero --> edge = wiring --> rimuovo edges(target_vertex, tree) 
                            edge_delete = set()
                            for v, x in self.Tree.E: #controllo gli edges che arrivano in xm nell albero
                                if x == xm: #aggiungo edge a lista eliminabili
                                    edge_delete.add((v, x))
                            for edge in edge_delete: #rimuovo edges inutili da albero
                                self.Tree.E.remove(edge)
                        else:
                            #xm non era nell albero --> edge = expansion --> sposto edges da X_unconn a QV (lo metto in coda per expansion)
                            self.X_unconn.remove(xm) #rimuovo xm da X_unconn --> non devo piu processarlo
                            self.Tree.V.add(xm) #aggiungo xm a albero
                            self.Tree.QV.add(xm) #aggiungo xm in coda a QV (non ho ancora assegnato costo)
                        
                        self.g_F[xm] = self.gF(vm) + actual_cost #assegno a xm il suo costo nel dizionario
                        self.Tree.E.add((vm, xm)) #aggiungo edge (vm, xm) nell albero
                        xm.parent = vm #assegno vm come parent di xm
                        set_delete = set() #edges che non migliorano soluzione in QE
                        for v, x in self.Tree.QE:
                            #tutti gli edge (v, xm) con costo > nuovo cost_to_come(xm) --> aggiungo lista eliminabili
                            if x == xm and self.g_F[v] + self.calc_dist(v, xm) >= self.gF(xm):
                                set_delete.add((v, x))

                        #rimuovo edges eliminabili da QE
                        for edge in set_delete:
                            self.Tree.QE.remove(edge)
            else:
                #stima cost_to_come(x_goal) troppo alta --> svuoto e new batch (nessun altro edge puo migliorare soluzione)
                self.Tree.QE = set() #svuoto dict QE
                self.Tree.QV = set() #svuoto dict QV


            #plotto
            self.iteration = self.iteration + 1
            if self.animation_on:
                if self.iteration % 5 == 0:
                    self.animation(self.xCenter, self.gF(self.x_goal), self.cMin, self.theta)


        #plotto a fine batch
        if self.plot_on:
            if self.x_goal.parent is not None:
                self.PlotFinalPath(True) #plotto path finale e interpolato con B-spline 

        #salvo dati per benchmark --> tengo conto di distanza e tempo
        self.benchmark_list.append([time.time() - start_time, self.gF(self.x_goal)])

        return self.returnPath(steps)



    #filtro vertici cambiati
    def cleanTree(self, v):
        return self.utils.is_inside_obs(v)


    #elimino nodi precedenti (gia passati)
    def removePrevious(self, v):
        return self.gF(v) < self.gF(self.x_start)


    #riparo e per replanning
    def replan(self):

        #reinizializzo tutto
        self.Tree.QE = set()
        self.Tree.QV = set()
        self.Tree.E = set()
        self.X_unconn = set()
        self.X_unconn.add(self.x_goal)
        self.g_F.clear()

        #tolgo da dict campioni dietro current x_start da dict        
        self.Tree.V = set(filter(self.cleanTree, self.Tree.V))

        #sicura se cancello start o goal
        self.Tree.V.add(self.x_start)
        self.Tree.V.add(self.x_goal)
        self.g_F[self.x_start] = 0
        self.g_F[self.x_goal] = np.inf

        #resample nella nuovo regione selezionata con [m] samples
        self.X_unconn.update(self.Sample(self.m)) 
        self.Tree.V_old = {v for v in self.Tree.V}
        self.Tree.QV = {v for v in self.Tree.V}


    #aggiungo rumore per muovere ostacoli e testare environment
    def moveObstacles(self, dx, dy, x_est, delta):
        #muovo ostacoli circolari
        for o in self.obs_circle:
            #rumore random
            dxo = np.random.uniform(-1, 1)*dx
            dyo = np.random.uniform(-1, 1)*dy
            #controlla se non e' collisione immediata
            if math.hypot(x_est.S[0] - (o[0] + dxo), x_est.S[1] - (o[1] + dyo)) > o[2] + delta:
                #aggiorna centro ostacolo
                o[0] = o[0] + dxo
                o[1] = o[1] + dyo

        #muovo ostacoli rettangolari
        for o in self.obs_rectangle:
            #rumore random
            dxo = np.random.uniform(-1, 1)*dx
            dyo = np.random.uniform(-1, 1)*dy
            #controlla se non e' collisione immediata
            if 0 > x_est.S[0] - ((o[0] + dxo) - delta) <= o[2] + 2 * delta \
                or 0 > x_est.S[1] - ((o[1] + dyo) - delta) <= o[3] + 2 * delta:
                #aggiorna centro ostacolo
                o[0] = o[0] + dxo
                o[1] = o[1] + dyo

        #aggiorno ostacoli per utils
        self.utils.update_obs(self.obs_circle, self.obs_boundary, self.obs_rectangle)


    #ciclo in dietro di steps
    def cycle(self, x, steps):
        i=0
        if x == None:
            return True
        while x.parent and i < steps:
            i = i+1
            x = x.parent
        if x == self.x_start and i == steps:
            return False
        return True


    #ciclo su parent --> estraggo x, y del path
    def ExtractPath(self):
        node = self.x_goal
        path_x, path_y = [node.S[0]], [node.S[1]]
        while node.parent:
            node = node.parent
            path_x.append(node.S[0])
            path_y.append(node.S[1])
        return path_x, path_y


    #come ExtractPath ma piu comodo per coppie (x, y)
    def returnPath(self, steps):
        x = self.x_goal
        while x and self.cycle(x, steps):
            x = x.parent
        return x


    #prune --> aggiorno X_unconn, vertes_set, edge_set
    def Prune(self, cBest):
        #seleziono solo x in Xsample con stima costo minore di miglior costo reale corrente
        self.X_unconn = {x for x in self.X_unconn if self.f_estimated(x) < cBest}
        #seleziono solo vertici V nell albero con stima costo minore di miglior costo reale corrente
        self.Tree.V = {v for v in self.Tree.V if self.f_estimated(v) <= cBest} 
        #seleziono solo vertici edges (v, w) nell albero con stima costo (entrambi nodi) minore di miglior costo reale corrente
        self.Tree.E = {(v, w) for v, w in self.Tree.E
                       if self.f_estimated(v) <= cBest and self.f_estimated(w) <= cBest}
        #disconnetto da albero vertici con costo inf --> aggiungo a Xsample da riconnettere
        self.X_unconn.update({v for v in self.Tree.V if self.gF(v) == np.inf})
        self.Tree.V = {v for v in self.Tree.V if self.gF(v) < np.inf}

        return len(self.X_unconn)


    #ritorno costo come distanza tra start-end (inf se ostacolo)
    def cost(self, start, end):
        if self.utils.is_collision(start, end):
            return np.inf
        return self.calc_dist(start, end)


    #stima cost_to_come(x) + cost_to_go(x) --> costo path
    def f_estimated(self, node):
        return self.g_estimated(node) + self.h_estimated(node)


    #stima cost_to_come(x) --> alzo le euristiche per replan
    def g_estimated(self, node):
        return self.calc_dist(self.x_start, node) * self.replan_factor


    #stima cost_to_go(x) --> alzo le euristiche per replan
    def h_estimated(self, node):
        return self.calc_dist(node, self.x_goal) * self.replan_factor


    #meglio cosi per errori
    def gF(self, v):
        if not v in self.g_F: #imporante! se non e nel dict --> inf
            return np.inf
        else:
            return self.g_F[v] #leggo valore nel dict


    #creo m nuovi sample Xrand
    def Sample(self, m):
        if self.cBest < np.inf: #se ho gia trovato un path --> stringo ellissoide
            return self.SampleEllipsoid(m, self.cBest, self.cMin, self.xCenter, self.C)
        else: #primo sample uniforme in c-space (posso subito applicare euristica volendo)
            return self.SampleFreeSpace(m)

    
    #plotto il path finale
    def PlotFinalPath(self, smooth):
        path_x, path_y = self.ExtractPath()
        if smooth and len(path_x) > 3: #se voglio interpolare con B-spline
            path_x, path_y = smoothing.approximate_b_spline_path(path_x, path_y, 4*len(path_x), degree=3)
        plt.plot(path_x, path_y, linewidth=2, color='r')
        plt.pause(0.1)


    #sample in regione definita da ellissoide
    def SampleEllipsoid(self, m, cMax, cMin, xCenter, C):
        r = [cMax / 2.0,
             math.sqrt(abs(cMax ** 2 - cMin ** 2)) / 2.0,
             math.sqrt(abs(cMax ** 2 - cMin ** 2)) / 2.0]
        L = np.diag(r)
        ind = 0
        Sample = set()
        #creo m sample
        while ind < m:
            xBall = self.SampleUnitNBall()
            x_rand = np.dot(np.dot(C, L), xBall) + xCenter
            node = Node((x_rand[(0, 0)], x_rand[(1, 0)])) #aggiungo covariance
            in_obs = self.utils.is_inside_obs(node) #check se sample interno a ostacolo
            in_x_range = self.x_range[0] <= node.S[0] <= self.x_range[1]
            in_y_range = self.y_range[0] <= node.S[1] <= self.y_range[1]
            #check se aggiungere o no
            if not in_obs and in_x_range and in_y_range:
                Sample.add(node)
                ind += 1
        return Sample


    #sampleFree normale
    def SampleFreeSpace(self, m):
        delta = self.utils.delta
        Sample = set()
        ind = 0
        while ind < m:
            node = Node((random.uniform(self.x_range[0], self.x_range[1] - delta),
                        random.uniform(self.y_range[0], self.y_range[1]) - delta))
            if self.utils.is_inside_obs(node):
                continue
            else:
                Sample.add(node)
                ind += 1
        return Sample


    #aggiorno raggio di ricerca
    def radius(self, q):
        cBest = self.g_F[self.x_goal]
        lambda_X = len([1 for v in self.Tree.V if self.f_estimated(v) <= cBest])
        radius = 2 * self.eta * (1.5 * lambda_X / math.pi * math.log(q) / q) ** 0.5
        return radius


    #dato vertice V in QV --> creo edges che aggiungo a QE
    def ExpandVertex(self, v):
        self.Tree.QV.remove(v) #selexiono V e lo rimuovo da QV
        X_near = {x for x in self.X_unconn if self.calc_dist(x, v) <= self.Tree.r} #trovo set vicini di V nel set di sample non connessi
        #se potenzialmente soluzione migliorabile --> aggiungo edge
        for x in X_near:
            #g_estimated(v) = cost to come (v) , calc_dist(v, x) = calcolo costo da V a x , h_estimated(x) = stima cost to go(x) , g_F[self.x_goal] = costo soluzione corrente nell albero [inf se non ce]
            if self.g_estimated(v) + self.calc_dist(v, x) + self.h_estimated(x) < self.gF(self.x_goal):
                self.g_F[x] = np.inf #inizialmente setto costo a inf (correggo dopo)
                self.Tree.QE.add((v, x)) #aggiungo (v, x) in coda a QE
        #se V non era gia nell'albero trovo vicini di V nell albero corrente
        if v and v not in self.Tree.V_old:
            V_near = {w for w in self.Tree.V if self.calc_dist(w, v) <= self.Tree.r}
            for w in V_near:
                #se (v, w) not in E (edge non compreso nell albero) --> se puo migliorare soluzione [sia X_goal che w] --> inserisco (v, w) in QE
                if (v, w) not in self.Tree.E and \
                        self.g_estimated(v) + self.calc_dist(v, w) + self.h_estimated(w) < self.gF(self.x_goal) and \
                        self.gF(v) + self.calc_dist(v, w) < self.gF(w):
                    self.Tree.QE.add((v, w))
                    #se vertice w non ha costo assegnato --> cost = inf
                    if w not in self.g_F:
                        self.g_F[w] = np.inf


    #ritorno stima costo minore vertice
    def BestVertexQueueValue(self):
        if not self.Tree.QV:
            return np.inf
        return min(self.gF(v) + self.h_estimated(v) for v in self.Tree.QV)


    #ritorno stima cost_to_come da V minore
    def BestEdgeQueueValue(self):
        if not self.Tree.QE:
            return np.inf
        return min(self.gF(v) + self.calc_dist(v, x) + self.h_estimated(x)
                   for v, x in self.Tree.QE)


    #ritorno miglior vertice in coda QV con ordine corrente
    def BestInVertexQueue(self):
        if not self.Tree.QV:
            print("QV is Empty!")
            return None
        v_value = {v: self.gF(v) + self.h_estimated(v) for v in self.Tree.QV}
        #leggo da dict QV key con val = min(cost)
        return min(v_value, key=v_value.get)


    #ritorno edge di QE con stima coso minore
    def BestInEdgeQueue(self):
        if not self.Tree.QE:
            print("QE is Empty!")
            return None, None
        e_value = {(v, x): self.gF(v) + self.calc_dist(v, x) + self.h_estimated(x)
                   for v, x in self.Tree.QE}
        #seleziono vertice con costo minimo dal dizionario
        return min(e_value, key=e_value.get)


    #sample in ball raggio 1x1 (in 2D)
    @staticmethod
    def SampleUnitNBall():
        while True:
            x, y = random.uniform(-1, 1), random.uniform(-1, 1)
            if x ** 2 + y ** 2 < 1:
                return np.array([[x], [y], [0.0]])


    #trovo direzione x_start-x_goal (direzione) --> aggiorno C
    def RotationToWorldFrame(self):
        #controllo per replanning
        if self.cMin < 1:
            return
        self.cMin, self.theta = self.calc_dist_and_angle(self.x_start, self.x_goal)
        a1 = np.array([[(self.x_goal.S[0] - self.x_start.S[0]) / self.cMin], [(self.x_goal.S[1] - self.x_start.S[1]) / self.cMin], [0.0]]) #vettore scalato x_start-->x_goal
        e1 = np.array([[1.0], [0.0], [0.0]])
        M = a1 @ e1.T #creo matrice 3x3 per applicare SVD
        U, _, V_T = np.linalg.svd(M, True, True) #SVD --> trovo direzioni principali per ruotare ellisse
        self.C = U @ np.diag([1.0, 1.0, np.linalg.det(U) * np.linalg.det(V_T.T)]) @ V_T


    #distanza tra due nodi
    @staticmethod
    def calc_dist(start, end):
        return math.hypot(start.S[0] - end.S[0], start.S[1] - end.S[1])


    #distanza e angolo tra due nodi
    @staticmethod
    def calc_dist_and_angle(node_start, node_end):
        dx = node_end.S[0] - node_start.S[0]
        dy = node_end.S[1] - node_start.S[1]
        return math.hypot(dx, dy), math.atan2(dy, dx)


    #definisco animation specifica di BIT* (non posso usare stessa di RRT...)
    def animation(self, xCenter, cMax, cMin, theta):
        plt.cla()
        self.plot_grid("Batch Informed Trees (BIT*)")
        plt.gcf().canvas.mpl_connect(
            'key_release_event',
            lambda event: [exit(0) if event.key == 'escape' else None])
        for v in self.X_unconn:
            plt.plot(v.S[0], v.S[1], marker='.', color='lightgrey', markersize='2')
        if cMax < np.inf:
            self.draw_ellipse(xCenter, cMax, cMin, theta)
        for v, w in self.Tree.E:
            plt.plot([v.S[0], w.S[0]], [v.S[1], w.S[1]], '-g')
        plt.pause(1e-5)


    #plotto tutto
    def plot_grid(self, name):
        for (ox, oy, w, h) in self.obs_rectangle:
            self.ax.add_patch(
                patches.Rectangle(
                    (ox, oy), w, h,
                    edgecolor='black',
                    facecolor='black',
                    fill=True
                )
            )
        for (ox, oy, r) in self.obs_circle:
            self.ax.add_patch(
                patches.Circle(
                    (ox, oy), r,
                    edgecolor='black',
                    facecolor='black',
                    fill=True
                )
            )
        plt.plot(self.x_start.S[0], self.x_start.S[1], "rs", linewidth=3)
        plt.plot(self.x_goal.S[0], self.x_goal.S[1], "rs", linewidth=3)
        plt.title(name)
        plt.axis("equal")


    #draw ellipse specifico per BIT*
    @staticmethod
    def draw_ellipse(x_center, c_best, dist, theta):
        a = math.sqrt(c_best ** 2 - dist ** 2) / 2.0
        b = c_best / 2.0
        angle = math.pi / 2.0 - theta
        cx = x_center[0]
        cy = x_center[1]
        t = np.arange(0, 2 * math.pi + 0.1, 0.2)
        x = [a * math.cos(it) for it in t]
        y = [b * math.sin(it) for it in t]
        rot = Rot.from_euler('z', -angle).as_matrix()[0:2, 0:2]
        fx = rot @ np.array([x, y])
        px = np.array(fx[0, :] + cx).flatten()
        py = np.array(fx[1, :] + cy).flatten()
        plt.plot(cx, cy, marker='.', color='darkorange')
        plt.plot(px, py, linestyle='--', color='darkorange', linewidth=2)


#=================== MAIN ==================================================
def main():

    #lista dove salvo i dati
    results = []
    tot_iter = 1

    #dati problema
    x_start = (5, 5)
    x_goal = (46, 28)
    steps = 4 #numero di step per volta
    iter = 0 #numero iterazioni

    #faccio iterazioni per salvare dati
    while iter < tot_iter:
        print(iter)
        #istanzio planner
        bit = BITStar(x_start, x_goal, 2, 0)
        x_est = bit.planning(steps = steps) #primo planning
        sample_density = bit.m / bit.calc_dist(bit.x_start, bit.x_goal) #densita samples in regione
        start_time = time.time()

        #ciclo fino a che non arrivo alla fine
        while x_est != None:

            #muovo ostacoli (no collisione immediata)
            bit.moveObstacles(3, 3, x_est, bit.utils.delta)
            
            bit.cMin = bit.calc_dist(x_est, bit.x_goal) #nuovo costo minimo
            bit.m = round(sample_density * bit.cMin) #nuovo numero samples regione piu piccola

            #rimuovo precedenti
            bit.Tree.V = set(filter(bit.removePrevious, bit.Tree.V))

            #aggiorno x_start
            bit.x_start = x_est
            bit.x_start.parent = None
            bit.Tree.V.add(bit.x_start)

            #aggiorno centro elllissoide
            bit.xCenter = np.array([[(bit.x_start.S[0] + bit.x_goal.S[0]) / 2.0],
                                    [(bit.x_start.S[1] + bit.x_goal.S[1]) / 2.0], [0.0]])
            #aggiorno cbest
            bit.cBest = bit.g_F[bit.x_goal] - bit.gF(x_est)

            #ruoto frame man mano che mi muovo
            bit.RotationToWorldFrame()

            #inizializzo replanning
            bit.replan()

            #corrisponde a ABITni del pseudocodice --> replanning
            x_est = bit.planning(steps)
            
            #sicura
            if not(bit.plot_on or bit.animation_on):
                if time.time()-start_time > 3:
                    break

        results.append(bit.benchmark_list)
        iter = iter + 1

    #salvo
    if tot_iter > 10:
        data_dir = os.path.join("../data", "replan.npy")
        np.save(data_dir, results)


if __name__ == '__main__':
    main()
