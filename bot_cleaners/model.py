from mesa.model import Model
from mesa.agent import Agent
from mesa.space import MultiGrid
from mesa.time import SimultaneousActivation
from mesa.datacollection import DataCollector

import numpy as np


class Celda(Agent):
    def __init__(self, unique_id, model, suciedad: bool = False):
        super().__init__(unique_id, model)
        self.sucia = suciedad


class Mueble(Agent):
    def __init__(self, unique_id, model):
        super().__init__(unique_id, model)


class CargadorRobot(Agent):

    def __init__(self, unique_id, model):
        super().__init__(unique_id, model)
        self.velocidad_carga = 99
        self.ocupado = False

    def cargar_robot(self, robot):
        if not self.ocupado and robot.carga < 100:
            self.ocupado = True
            robot.carga += self.velocidad_carga
            robot.carga = min(robot.carga, 100)
            


class RobotLimpieza(Agent):
    def __init__(self, unique_id, model):
        super().__init__(unique_id, model)
        self.sig_pos = None
        self.movimientos = 0
        self.carga = 100
        self.buscar_cargador = 30
        self.velocidad_carga = 25  # Cantidad de carga que se agrega en cada step
        self.cargando = False

    def limpiar_una_celda(self, lista_de_celdas_sucias):
        celda_a_limpiar = self.random.choice(lista_de_celdas_sucias)
        celda_a_limpiar.sucia = False
        self.sig_pos = celda_a_limpiar.pos

    def seleccionar_nueva_pos(self, lista_de_vecinos):
        # Obtener todas las posiciones disponibles
        posiciones_disponibles = [vecino.pos for vecino in lista_de_vecinos]

        # Excluir la posición actual del robot de las posiciones disponibles
        posiciones_disponibles = [pos for pos in posiciones_disponibles if pos != self.pos]

        # Excluir la posición siguiente de otros robots
        posiciones_sig_otro_robot = [robot.sig_pos for robot in self.model.schedule.agents if isinstance(robot, RobotLimpieza) and robot.unique_id != self.unique_id]
        posiciones_disponibles = [pos for pos in posiciones_disponibles if pos not in posiciones_sig_otro_robot]

        # Seleccionar una nueva posición de forma aleatoria
        self.sig_pos = self.random.choice(posiciones_disponibles)

    @staticmethod
    def buscar_celdas_sucia(lista_de_vecinos):
        # #Opción 1
        # return [vecino for vecino in lista_de_vecinos
        #                 if isinstance(vecino, Celda) and vecino.sucia]
        # #Opción 2
        celdas_sucias = list()
        for vecino in lista_de_vecinos:
            if isinstance(vecino, Celda) and vecino.sucia:
                celdas_sucias.append(vecino)
        return celdas_sucias

    def mover_hacia_cargador(self):
        cargador_mas_cercano = self.encontrar_cargador_mas_cercano()
        self.mover_un_paso_hacia(cargador_mas_cercano)

    def encontrar_cargador_mas_cercano(self):
        # Identificar la ubicación de todas las estaciones de carga
        posiciones_cargadores = [cargador.pos for cargador in self.model.schedule.agents
                                 if isinstance(cargador, CargadorRobot)]

        # Encontrar el cargador más cercano
        cargador_mas_cercano = min(posiciones_cargadores,
                                   key=lambda pos: self.distancia(self.pos, pos))
        return cargador_mas_cercano

    def mover_un_paso_hacia(self, destino):
        dx = destino[0] - self.pos[0]
        dy = destino[1] - self.pos[1]

        if abs(dx) > abs(dy):  # Moverse en el eje X
            nueva_x = self.pos[0] + (1 if dx > 0 else -1)
            nueva_y = self.pos[1]
        else:  # Moverse en el eje Y
            nueva_x = self.pos[0]
            nueva_y = self.pos[1] + (1 if dy > 0 else -1)

        # Verificar si la nueva posición está dentro de los límites
        nueva_pos = (nueva_x, nueva_y)
        if 0 <= nueva_x < self.model.grid.width and 0 <= nueva_y < self.model.grid.height and nueva_pos != self.sig_pos:
            # Verificar que la nueva posición no sea la posición siguiente de otros robots
            posiciones_sig_otro_robot = [robot.sig_pos for robot in self.model.schedule.agents if isinstance(robot, RobotLimpieza) and robot.unique_id != self.unique_id]
            if nueva_pos not in posiciones_sig_otro_robot:
                self.sig_pos = nueva_pos
                self.model.grid.move_agent(self, self.sig_pos)
                self.movimientos += 1

    def distancia(self, pos1, pos2):
        # Calcular la distancia de Manhattan
        return abs(pos1[0] - pos2[0]) + abs(pos1[1] - pos2[1])

    def step(self):
        vecinos = self.model.grid.get_neighbors(
            self.pos, moore=True, include_center=False)

        for vecino in vecinos:
            if isinstance(vecino, (Mueble, RobotLimpieza)):
                vecinos.remove(vecino)

        celdas_sucias = self.buscar_celdas_sucia(vecinos)

        if not self.cargando and self.carga < self.buscar_cargador:
            self.mover_hacia_cargador()

            cargador = self.obtener_cargador_en_posicion()
            if cargador:
                self.cargando = True

        elif self.cargando:
            # El robot está cargando, agregar carga en cada step hasta alcanzar la capacidad máxima
            self.carga = min(100, self.carga + self.velocidad_carga)

            # Si la carga alcanza la capacidad máxima, dejar de cargar
            if self.carga == 100:
                self.cargando = False

        elif len(celdas_sucias) == 0:
            self.seleccionar_nueva_pos(vecinos)
        else:
            self.limpiar_una_celda(celdas_sucias)

        self.carga -= 1

    def obtener_cargador_en_posicion(self):
        # Buscar un cargador en la misma posición que este robot
        celda_actual = self.model.grid.get_cell_list_contents([self.pos])
        cargadores = [obj for obj in celda_actual if isinstance(obj, CargadorRobot)]
        return cargadores[0] if cargadores else None
    
    def advance(self):
        if self.pos != self.sig_pos:
            self.movimientos += 1

        if self.carga > 0 and not self.cargando:
            self.carga -= 1
            self.model.grid.move_agent(self, self.sig_pos)
        elif self.carga == 0 and not self.cargando:
            # Si la carga llega a cero, iniciar proceso de carga
            self.cargando = True

class Habitacion(Model):
    def __init__(self, M: int, N: int,
                 num_agentes: int = 5,
                 porc_celdas_sucias: float = 0.6,
                 porc_muebles: float = 0.1,
                 modo_pos_inicial: str = 'Fija',
                 num_cargadores: int = 4,
                 ):
        self.unique_id_counter = 0
        self.num_agentes = num_agentes
        self.porc_celdas_sucias = porc_celdas_sucias
        self.porc_muebles = porc_muebles
        self.num_cargadores = num_cargadores

        self.grid = MultiGrid(M, N, False)
        self.schedule = SimultaneousActivation(self)

        posiciones_disponibles = [pos for _, pos in self.grid.coord_iter()]

        mitad_ancho = self.grid.width // 2
        mitad_altura = self.grid.height // 2
        posiciones_cargadores = [
            (mitad_ancho // 2, mitad_altura // 2),  # Cerca del centro en el cuadrante superior izquierdo
            (mitad_ancho // 2, 3 * mitad_altura // 2),  # Cerca del centro en el cuadrante inferior izquierdo
            (3 * mitad_ancho // 2, mitad_altura // 2),  # Cerca del centro en el cuadrante superior derecho
            (3 * mitad_ancho // 2, 3 * mitad_altura // 2)  # Cerca del centro en el cuadrante inferior derecho
        ]

        for id, posicion in enumerate(posiciones_cargadores):
            cargador = CargadorRobot(self.unique_id_counter, self)
            self.unique_id_counter += 1
            self.grid.place_agent(cargador, posicion)
            self.schedule.add(cargador)

        # Posicionamiento de muebles
        num_muebles = int(M * N * porc_muebles)
        posiciones_muebles = self.random.sample(posiciones_disponibles, k=num_muebles)

        for id, pos in enumerate(posiciones_muebles):
            mueble = Mueble(int(f"{num_agentes}0{id}") + 1, self)
            self.grid.place_agent(mueble, pos)
            posiciones_disponibles.remove(pos)

        # Posicionamiento de celdas sucias
        self.num_celdas_sucias = int(M * N * porc_celdas_sucias)
        posiciones_celdas_sucias = self.random.sample(
            posiciones_disponibles, k=self.num_celdas_sucias)

        for id, pos in enumerate(posiciones_disponibles):
            suciedad = pos in posiciones_celdas_sucias
            celda = Celda(int(f"{num_agentes}{id}") + 1, self, suciedad)
            self.grid.place_agent(celda, pos)

        # Posicionamiento de agentes robot
        if modo_pos_inicial == 'Aleatoria':
            pos_inicial_robots = self.random.sample(posiciones_disponibles, k=num_agentes)
        else:  # 'Fija'
            pos_inicial_robots = [(1, 1)] * num_agentes

        for id in range(num_agentes):
            # Usar el contador de IDs únicos
            robot = RobotLimpieza(self.unique_id_counter, self)
            self.unique_id_counter += 1  # Incrementar el contador
            self.grid.place_agent(robot, pos_inicial_robots[id])
            self.schedule.add(robot)

        self.datacollector = DataCollector(
            model_reporters={"Grid": get_grid, "Cargas": get_cargas,
                             "CeldasSucias": get_sucias},
        )

    def step(self):
        self.datacollector.collect(self)

        self.schedule.step()

    def todoLimpio(self):
        for (content, x, y) in self.grid.coord_iter():
            for obj in content:
                if isinstance(obj, Celda) and obj.sucia:
                    return False
        return True


def get_grid(model: Model) -> np.ndarray:
    """
    Método para la obtención de la grid y representarla en un notebook
    :param model: Modelo (entorno)
    :return: grid
    """
    grid = np.zeros((model.grid.width, model.grid.height))
    for cell in model.grid.coord_iter():
        cell_content, pos = cell
        x, y = pos
        for obj in cell_content:
            if isinstance(obj, RobotLimpieza):
                grid[x][y] = 2
            elif isinstance(obj, Celda):
                grid[x][y] = int(obj.sucia)
    return grid


def get_cargas(model: Model):
    return [(agent.unique_id, agent.carga) for agent in model.schedule.agents if isinstance(agent, RobotLimpieza)]


def get_sucias(model: Model) -> int:
    """
    Método para determinar el número total de celdas sucias
    :param model: Modelo Mesa
    :return: número de celdas sucias
    """
    sum_sucias = 0
    for cell in model.grid.coord_iter():
        cell_content, pos = cell
        for obj in cell_content:
            if isinstance(obj, Celda) and obj.sucia:
                sum_sucias += 1
    return sum_sucias / model.num_celdas_sucias


def get_movimientos(agent: Agent) -> dict:
    if isinstance(agent, RobotLimpieza):
        return {agent.unique_id: agent.movimientos}
    # else:
    #    return 0
