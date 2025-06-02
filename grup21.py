import random
import datetime
import time
from typing import List, Tuple, Dict, Set
import math
import heapq
import matplotlib.pyplot as plt
import numpy as np
from matplotlib.patches import Polygon


# -------------------------
# 1. Veri Yapıları
# -------------------------

class Drone:
    def __init__(self, id: int, max_weight: float, battery: int, speed: float, start_pos: Tuple[float, float]):
        self.id = id
        self.max_weight = max_weight  # kg cinsinden
        self.battery = battery  # mAh cinsinden
        self.speed = speed  # m/s cinsinden
        self.start_pos = start_pos
        self.current_pos = start_pos
        self.remaining_battery = battery
        self.total_distance = 0
        self.total_energy_used = 0
        self.deliveries_completed = 0

    def __str__(self):
        return f"Drone {self.id} (Max Ağırlık: {self.max_weight}kg, Batarya: {self.battery}mAh, Hız: {self.speed}m/s)"


class DeliveryPoint:
    def __init__(self, id: int, pos: Tuple[float, float], weight: float, priority: int,
                 time_window: Tuple[datetime.time, datetime.time]):
        self.id = id
        self.pos = pos
        self.weight = weight  # kg cinsinden
        self.priority = priority  # 1: düşük, 5: yüksek
        self.time_window = time_window
        self.delivered = False

    def __eq__(self, other):
        if not isinstance(other, DeliveryPoint):
            return False
        return self.id == other.id

    def __hash__(self):
        return hash(self.id)

    def __str__(self):
        return f"Teslimat {self.id} (Konum: {self.pos}, Ağırlık: {self.weight}kg, Öncelik: {self.priority})"


class NoFlyZone:
    def __init__(self, id: int, coordinates: List[Tuple[float, float]],
                 active_time: Tuple[datetime.time, datetime.time]):
        self.id = id
        self.coordinates = coordinates
        self.active_time = active_time

    def __str__(self):
        return f"No-Fly Zone {self.id} (Köşeler: {len(self.coordinates)})"


# -------------------------
# 2. Yardımcı Fonksiyonlar
# -------------------------

def euclidean_distance(p1: Tuple[float, float], p2: Tuple[float, float]) -> float:
    """İki nokta arasındaki Öklid mesafesini hesaplar"""
    return math.sqrt((p1[0] - p2[0]) ** 2 + (p1[1] - p2[1]) ** 2)


def point_in_polygon(point: Tuple[float, float], polygon: List[Tuple[float, float]]) -> bool:
    """Ray casting algoritması ile bir noktanın polygon içinde olup olmadığını kontrol eder"""
    x, y = point
    n = len(polygon)
    inside = False
    p1x, p1y = polygon[0]
    for i in range(n + 1):
        p2x, p2y = polygon[i % n]
        if y > min(p1y, p2y):
            if y <= max(p1y, p2y):
                if x <= max(p1x, p2x):
                    if p1y != p2y:
                        xinters = (y - p1y) * (p2x - p1x) / (p2y - p1y) + p1x
                    if p1x == p2x or x <= xinters:
                        inside = not inside
        p1x, p1y = p2x, p2y
    return inside


def is_in_no_fly_zone(point: Tuple[float, float], time: datetime.time, noflyzones: List[NoFlyZone]) -> bool:
    """Bir noktanın herhangi bir no-fly zone içinde olup olmadığını kontrol eder"""
    for zone in noflyzones:
        if zone.active_time[0] <= time <= zone.active_time[1]:
            if point_in_polygon(point, zone.coordinates):
                return True
    return False


def line_segment_intersects_edge(p1: Tuple[float, float], p2: Tuple[float, float],
                                 e1: Tuple[float, float], e2: Tuple[float, float]) -> bool:
    """İki çizgi parçasının kesişip kesişmediğini kontrol eder"""

    def orientation(p, q, r):
        val = (q[1] - p[1]) * (r[0] - q[0]) - (q[0] - p[0]) * (r[1] - q[1])
        if val == 0:
            return 0  # collinear
        return 1 if val > 0 else 2  # clock or counterclock wise

    def on_segment(p, q, r):
        return (q[0] <= max(p[0], r[0]) and q[0] >= min(p[0], r[0]) and
                q[1] <= max(p[1], r[1]) and q[1] >= min(p[1], r[1]))

    o1 = orientation(p1, p2, e1)
    o2 = orientation(p1, p2, e2)
    o3 = orientation(e1, e2, p1)
    o4 = orientation(e1, e2, p2)

    # Genel durum
    if o1 != o2 and o3 != o4:
        return True

    # Özel durumlar
    if o1 == 0 and on_segment(p1, e1, p2): return True
    if o2 == 0 and on_segment(p1, e2, p2): return True
    if o3 == 0 and on_segment(e1, p1, e2): return True
    if o4 == 0 and on_segment(e1, p2, e2): return True

    return False


def line_intersects_polygon(p1: Tuple[float, float], p2: Tuple[float, float],
                            polygon: List[Tuple[float, float]]) -> bool:
    """Bir çizginin polygon ile kesişip kesişmediğini kontrol eder"""
    n = len(polygon)
    for i in range(n):
        if line_segment_intersects_edge(p1, p2, polygon[i], polygon[(i + 1) % n]):
            return True
    return False


def is_path_in_no_fly_zone(p1: Tuple[float, float], p2: Tuple[float, float], time: datetime.time,
                           noflyzones: List[NoFlyZone]) -> bool:
    """Bir yolun no-fly zone içinden geçip geçmediğini kontrol eder"""
    # Hem p1 veya p2 noktası no-fly zone içinde mi?
    if is_in_no_fly_zone(p1, time, noflyzones) or is_in_no_fly_zone(p2, time, noflyzones):
        return True
    # Aradaki çizgi no-fly zone ile kesişiyor mu?
    for zone in noflyzones:
        if zone.active_time[0] <= time <= zone.active_time[1]:
            if line_intersects_polygon(p1, p2, zone.coordinates):
                return True
    return False


def calculate_edge_cost(start: Tuple[float, float], end: Tuple[float, float],
                        delivery_priority: int, speed: float) -> float:
    """
    PDF'deki formüle göre kenar maliyetini hesaplar:
    Maliyet = Uzaklık × Hız + (Öncelik × 100)
    """
    distance = euclidean_distance(start, end)
    return distance * speed + (delivery_priority * 100)


def calculate_energy_consumption(distance: float, weight: float, max_weight: float) -> float:
    """Enerji tüketimini hesaplar"""
    # Basit bir enerji modeli: mesafe ve taşınan ağırlığa bağlı
    return distance * (1 + weight / max_weight)


# -------------------------
# 3. A* Algoritması (PDF'deki formüle göre)
# -------------------------

class Node:
    def __init__(self, pos: Tuple[float, float], g=0, h=0, parent=None):
        self.pos = pos
        self.g = g  # Başlangıçtan bu noktaya kadar olan maliyet
        self.h = h  # Tahmini kalan maliyet (heuristic)
        self.f = g + h  # Toplam maliyet
        self.parent = parent

    def __lt__(self, other):
        return self.f < other.f

    def __eq__(self, other):
        return isinstance(other, Node) and self.pos == other.pos

    def __hash__(self):
        return hash(self.pos)


def a_star(start: Tuple[float, float], goal: Tuple[float, float],
           noflyzones: List[NoFlyZone], current_time: datetime.time,
           delivery_priority: int = 1, speed: float = 10.0,
           grid_size=100, step_size=2.0) -> List[Tuple[float, float]]:
    """
    A* algoritması ile iki nokta arasında no-fly zone'lardan kaçınan bir yol bulur
    PDF'deki formüle göre: f = g + h
    g = Başlangıçtan bu noktaya kadar olan maliyet
    h = Tahmini kalan maliyet + no-fly zone cezası
    """
    open_set = []
    closed_set = set()

    start_node = Node(start)
    start_node.h = euclidean_distance(start, goal)

    heapq.heappush(open_set, (start_node.f, id(start_node), start_node))
    node_dict = {start: start_node}

    max_iterations = 1000  # Sonsuz döngüyü önlemek için
    iterations = 0

    while open_set and iterations < max_iterations:
        iterations += 1
        _, _, current_node = heapq.heappop(open_set)

        # Hedefe ulaştık mı?
        if euclidean_distance(current_node.pos, goal) < step_size:
            path = []
            while current_node:
                path.append(current_node.pos)
                current_node = current_node.parent
            return path[::-1]  # Yolu tersine çevir (başlangıçtan hedefe)

        closed_set.add(current_node.pos)

        # 8 yönde hareket (kuzey, güney, doğu, batı ve köşegenler)
        directions = [(-1, 0), (1, 0), (0, -1), (0, 1), (-1, -1), (-1, 1), (1, -1), (1, 1)]
        for dx, dy in directions:
            neighbor_pos = (current_node.pos[0] + dx * step_size, current_node.pos[1] + dy * step_size)

            # Sınırlar içinde mi?
            if not (0 <= neighbor_pos[0] <= grid_size and 0 <= neighbor_pos[1] <= grid_size):
                continue

            # No-fly zone kontrolü
            if is_path_in_no_fly_zone(current_node.pos, neighbor_pos, current_time, noflyzones):
                continue

            if neighbor_pos in closed_set:
                continue

            # PDF'deki formüle göre g değeri hesaplama
            edge_cost = calculate_edge_cost(current_node.pos, neighbor_pos, delivery_priority, speed)
            tentative_g = current_node.g + edge_cost

            if neighbor_pos in node_dict:
                neighbor_node = node_dict[neighbor_pos]
                if tentative_g >= neighbor_node.g:
                    continue
            else:
                neighbor_node = Node(neighbor_pos)
                node_dict[neighbor_pos] = neighbor_node

            neighbor_node.parent = current_node
            neighbor_node.g = tentative_g

            # PDF'deki formüle göre h değeri (no-fly zone cezası eklenmiş)
            base_h = euclidean_distance(neighbor_pos, goal)
            no_fly_penalty = 0

            # Eğer doğrudan hedefe giden yol no-fly zone içinden geçiyorsa ceza ekle
            if is_path_in_no_fly_zone(neighbor_pos, goal, current_time, noflyzones):
                no_fly_penalty = 1000  # Büyük bir ceza değeri

            neighbor_node.h = base_h + no_fly_penalty
            neighbor_node.f = neighbor_node.g + neighbor_node.h

            if neighbor_pos not in [n.pos for _, _, n in open_set]:
                heapq.heappush(open_set, (neighbor_node.f, id(neighbor_node), neighbor_node))

    # Eğer yol bulunamazsa, doğrudan bir çizgi çizmeyi dene
    if not is_path_in_no_fly_zone(start, goal, current_time, noflyzones):
        return [start, goal]

    # Hiçbir yol bulunamadı
    return []


# -------------------------
# 4. CSP ve Genetik Algoritma
# -------------------------

class Route:
    def __init__(self, drone: Drone, deliveries: List[DeliveryPoint] = None):
        self.drone = drone
        self.deliveries = deliveries if deliveries else []
        self.path = []  # Tam yol (tüm noktalar)
        self.waypoints = []  # Sadece teslimat noktaları
        self.cost = float('inf')
        self.energy_consumption = 0
        self.total_distance = 0
        self.total_priority = 0
        self.valid = True  # Rota geçerli mi?

    def add_delivery(self, delivery: DeliveryPoint) -> bool:
        """Rotaya bir teslimat noktası ekler, eğer ağırlık sınırını aşarsa False döner"""
        current_weight = sum(d.weight for d in self.deliveries)
        if current_weight + delivery.weight > self.drone.max_weight:
            return False
        self.deliveries.append(delivery)
        return True

    def calculate_path(self, noflyzones: List[NoFlyZone], current_time: datetime.time):
        """Rota için tam yolu hesaplar"""
        if not self.deliveries:
            self.path = []
            self.waypoints = []
            self.cost = 0
            self.energy_consumption = 0
            self.total_distance = 0
            self.total_priority = 0
            self.valid = True
            return

        # Teslimat noktalarını önceliğe göre sırala (yüksek öncelik önce)
        self.deliveries.sort(key=lambda d: d.priority, reverse=True)

        # Yol noktalarını oluştur (başlangıç -> teslimatlar -> başlangıç)
        self.waypoints = [self.drone.start_pos]
        for delivery in self.deliveries:
            self.waypoints.append(delivery.pos)
        self.waypoints.append(self.drone.start_pos)  # Başlangıç noktasına dön

        full_path = []
        total_distance = 0
        total_cost = 0
        total_energy = 0
        total_priority = sum(d.priority for d in self.deliveries)

        # Her bir segment için A* ile yol bul
        for i in range(len(self.waypoints) - 1):
            # Eğer bu bir teslimat noktasıysa, önceliği al
            delivery_priority = 1
            if i > 0 and i <= len(self.deliveries):
                delivery_priority = self.deliveries[i - 1].priority

            segment = a_star(
                self.waypoints[i],
                self.waypoints[i + 1],
                noflyzones,
                current_time,
                delivery_priority,
                self.drone.speed
            )

            if not segment:  # Eğer yol bulunamazsa
                self.path = []
                self.cost = float('inf')
                self.energy_consumption = float('inf')
                self.total_distance = 0
                self.total_priority = 0
                self.valid = False
                return

            # İlk nokta hariç ekle (çakışmayı önlemek için)
            if i > 0:
                segment = segment[1:]

            full_path.extend(segment)

            # Segment mesafesini hesapla
            segment_distance = 0
            for j in range(len(segment) - 1):
                segment_distance += euclidean_distance(segment[j], segment[j + 1])

            total_distance += segment_distance

            # Segment maliyetini hesapla (PDF'deki formüle göre)
            segment_cost = calculate_edge_cost(
                self.waypoints[i],
                self.waypoints[i + 1],
                delivery_priority,
                self.drone.speed
            )
            total_cost += segment_cost

            # Segment enerji tüketimini hesapla
            segment_weight = 0
            if i < len(self.deliveries):
                segment_weight = self.deliveries[i].weight

            segment_energy = calculate_energy_consumption(
                segment_distance,
                segment_weight,
                self.drone.max_weight
            )
            total_energy += segment_energy

        self.path = full_path
        self.cost = total_cost
        self.energy_consumption = total_energy
        self.total_distance = total_distance
        self.total_priority = total_priority
        self.valid = True

    def __str__(self):
        return (f"Drone {self.drone.id} Rotası: {len(self.deliveries)} teslimat, "
                f"Maliyet: {self.cost:.2f}, Enerji: {self.energy_consumption:.2f}")


def check_constraints(route: Route, noflyzones: List[NoFlyZone], current_time: datetime.time) -> bool:
    """Rotanın kısıtlamaları karşılayıp karşılamadığını kontrol eder (CSP)"""
    # Max ağırlık kontrolü
    total_weight = sum(d.weight for d in route.deliveries)
    if total_weight > route.drone.max_weight:
        return False

    # Zaman penceresi kontrolü
    now = datetime.datetime.now().time()
    for dp in route.deliveries:
        if not (dp.time_window[0] <= now <= dp.time_window[1]):
            return False

    # Yol hesapla ve no-fly zone kontrolü yap
    route.calculate_path(noflyzones, current_time)
    if not route.valid or route.cost == float('inf'):
        return False

    # Enerji kontrolü
    if route.energy_consumption > route.drone.battery / 1000:  # mAh'ı basit bir şekilde enerji birimine dönüştürme
        return False

    return True


def fitness(routes: List[Route], noflyzones: List[NoFlyZone], current_time: datetime.time) -> float:
    """
    Rotaların uygunluk değerini hesaplar - PDF'deki formüle göre:
    fitness = tamamlanan teslimat sayısı × 100 - (enerji tüketimi × 0.5) - (kural ihlali sayısı × 2000)
    """
    total_deliveries = 0
    total_energy = 0
    no_fly_violations = 0

    for route in routes:
        if not check_constraints(route, noflyzones, current_time):
            no_fly_violations += 1
            continue

        total_deliveries += len(route.deliveries)
        total_energy += route.energy_consumption

    fitness_value = (total_deliveries * 100) - (total_energy * 0.5) - (no_fly_violations * 2000)
    return fitness_value


def crossover(parent1: List[Route], parent2: List[Route], drones: List[Drone]) -> List[Route]:
    """İki ebeveyn rotadan yeni bir çocuk rota oluşturur"""
    child = []

    # Her drone için yeni bir rota oluştur
    for i in range(len(drones)):
        p1_deliveries = parent1[i].deliveries.copy() if i < len(parent1) else []
        p2_deliveries = parent2[i].deliveries.copy() if i < len(parent2) else []

        # Rastgele bir kesme noktası seç
        split = random.randint(0, len(p1_deliveries))

        # Çocuk teslimatları oluştur
        child_deliveries = p1_deliveries[:split]

        # P2'den eklerken zaten varsa ekleme (teslimat tekrarını önlemek için)
        for dp in p2_deliveries:
            if dp not in child_deliveries:
                child_deliveries.append(dp)

        child.append(Route(drones[i], child_deliveries))

    return child


def mutate(routes: List[Route], all_deliveries: List[DeliveryPoint], mutation_rate=0.1):
    """Rotaları belirli bir olasılıkla mutasyona uğratır"""
    # Tüm teslimatları toplama
    all_route_deliveries = set()
    for route in routes:
        for delivery in route.deliveries:
            all_route_deliveries.add(delivery)

    # Henüz atanmamış teslimatlar
    unassigned_deliveries = [d for d in all_deliveries if d not in all_route_deliveries]

    for route in routes:
        # 1. Teslimat noktalarını karıştır
        if random.random() < mutation_rate and len(route.deliveries) > 1:
            idx1, idx2 = random.sample(range(len(route.deliveries)), 2)
            route.deliveries[idx1], route.deliveries[idx2] = route.deliveries[idx2], route.deliveries[idx1]

        # 2. Bazen bir teslimatı rastgele bir rotadan diğerine taşı
        if random.random() < mutation_rate / 2:
            other_routes = [r for r in routes if r != route]
            if other_routes and route.deliveries:
                target_route = random.choice(other_routes)
                if route.deliveries:
                    delivery = random.choice(route.deliveries)
                    route.deliveries.remove(delivery)
                    target_route.deliveries.append(delivery)

        # 3. Bazen yeni bir teslimat ekle (eğer atanmamış teslimat varsa)
        if random.random() < mutation_rate / 3 and unassigned_deliveries:
            delivery = random.choice(unassigned_deliveries)
            if route.add_delivery(delivery):  # Eğer ağırlık sınırını aşmazsa ekle
                unassigned_deliveries.remove(delivery)

        # 4. Bazen bir teslimatı kaldır
        if random.random() < mutation_rate / 4 and route.deliveries:
            delivery = random.choice(route.deliveries)
            route.deliveries.remove(delivery)
            unassigned_deliveries.append(delivery)

    # Kalan atanmamış teslimatları rastgele dağıt
    random.shuffle(unassigned_deliveries)
    for delivery in unassigned_deliveries:
        random.shuffle(routes)  # Rotaları karıştır
        for route in routes:
            if route.add_delivery(delivery):
                break


def genetic_algorithm(drones: List[Drone], deliveries: List[DeliveryPoint], noflyzones: List[NoFlyZone],
                      generations=50, population_size=30):
    """Genetik algoritma ile rota optimizasyonu yapar"""
    current_time = datetime.datetime.now().time()
    start_time = time.time()

    # Başlangıç popülasyonu
    population = []
    for _ in range(population_size):
        # Teslimatları karıştır
        random.shuffle(deliveries)

        # Her drone'a teslimatları dağıt
        routes = []
        remaining_deliveries = deliveries.copy()

        for drone in drones:
            route = Route(drone)
            routes.append(route)

            # Bu drone için rastgele sayıda teslimat seç
            num_deliveries = random.randint(0, min(5, len(remaining_deliveries)))
            for _ in range(num_deliveries):
                if remaining_deliveries:
                    delivery = random.choice(remaining_deliveries)
                    if route.add_delivery(delivery):
                        remaining_deliveries.remove(delivery)

        # Kalan teslimatları rastgele dağıt
        random.shuffle(remaining_deliveries)
        for delivery in remaining_deliveries:
            random.shuffle(routes)  # Rotaları karıştır
            for route in routes:
                if route.add_delivery(delivery):
                    break

        population.append(routes)

    best_fitness = float('-inf')
    best_solution = None

    for gen in range(generations):
        # Her rotanın yolunu hesapla ve uygunluk değerini bul
        scored_population = []
        for routes in population:
            fit = fitness(routes, noflyzones, current_time)
            scored_population.append((fit, routes))

            if fit > best_fitness:
                best_fitness = fit
                best_solution = routes

        # En iyi yarısını seç
        scored_population.sort(key=lambda x: x[0], reverse=True)
        population = [x[1] for x in scored_population[:population_size // 2]]

        # Yeni popülasyon oluştur
        new_population = []
        while len(new_population) < population_size:
            # Turnuva seçimi
            parents = random.sample(population, min(2, len(population)))
            child = crossover(parents[0], parents[1], drones)
            mutate(child, deliveries, mutation_rate=0.2)
            new_population.append(child)

        population = new_population

        if gen % 10 == 0 or gen == generations - 1:
            print(f"Nesil {gen + 1}: En İyi Uygunluk = {best_fitness:.2f}")

    # En iyi çözüm için yolları hesapla
    for route in best_solution:
        route.calculate_path(noflyzones, current_time)

    end_time = time.time()
    algorithm_time = end_time - start_time
    print(f"Algoritma çalışma süresi: {algorithm_time:.2f} saniye")

    return best_solution, algorithm_time


# -------------------------
# 5. Veri Üreteci
# -------------------------

def random_time_window():
    """Rastgele bir zaman penceresi oluşturur"""
    start_hour = random.randint(9, 15)
    start = datetime.time(start_hour, 0)
    end = datetime.time(start_hour + 1, 0)
    return (start, end)


def generate_random_drones(num=5) -> List[Drone]:
    """Rastgele drone'lar oluşturur"""
    drones = []
    for i in range(num):
        drones.append(Drone(
            id=i,
            max_weight=random.uniform(5, 15),
            battery=random.randint(8000, 12000),
            speed=random.uniform(5, 15),
            start_pos=(random.uniform(0, 100), random.uniform(0, 100))
        ))
    return drones


def generate_random_deliveries(num=20) -> List[DeliveryPoint]:
    """Rastgele teslimat noktaları oluşturur"""
    deliveries = []
    for i in range(num):
        deliveries.append(DeliveryPoint(
            id=i,
            pos=(random.uniform(0, 100), random.uniform(0, 100)),
            weight=random.uniform(0.1, 5),
            priority=random.randint(1, 5),
            time_window=random_time_window()
        ))
    return deliveries


def generate_random_noflyzones(num=2) -> List[NoFlyZone]:
    """Rastgele no-fly zone'lar oluşturur"""
    zones = []
    for i in range(num):
        x, y = random.uniform(10, 90), random.uniform(10, 90)
        size = random.uniform(5, 15)
        coords = [(x, y), (x + size, y), (x + size, y + size), (x, y + size)]
        active_time = (datetime.time(0, 0), datetime.time(23, 59))  # Tüm gün aktif
        zones.append(NoFlyZone(id=i, coordinates=coords, active_time=active_time))
    return zones


# -------------------------
# 6. Görselleştirme
# -------------------------

def plot_routes(drones_routes: List[Route], noflyzones: List[NoFlyZone], deliveries: List[DeliveryPoint],
                title="Drone Teslimat Rotaları"):
    """Drone rotalarını ve no-fly zone'ları görselleştirir"""
    plt.figure(figsize=(12, 10))

    # No-fly zone'ları çiz
    for zone in noflyzones:
        xs, ys = zip(*zone.coordinates)
        xs = list(xs) + [xs[0]]  # Poligonu kapatmak için
        ys = list(ys) + [ys[0]]
        plt.fill(xs, ys, 'r', alpha=0.3)
        plt.plot(xs, ys, 'r-')

    # Tüm teslimat noktalarını topla
    all_delivery_ids = set()
    for route in drones_routes:
        for delivery in route.deliveries:
            all_delivery_ids.add(delivery.id)

    # Teslim edilmemiş noktaları çiz (sarı)
    for delivery in deliveries:
        if delivery.id not in all_delivery_ids:
            plt.plot(delivery.pos[0], delivery.pos[1], 'o', color='y', markersize=8)

    # Drone rotalarını çiz
    colors = ['b', 'g', 'm', 'c']
    for idx, route in enumerate(drones_routes):
        if not route.valid or not route.path:
            continue

        color = colors[idx % len(colors)]

        # Drone başlangıç noktasını çiz
        plt.plot(route.drone.start_pos[0], route.drone.start_pos[1], 'o', color=color, markersize=10)

        # Teslimat noktalarını çiz
        for delivery in route.deliveries:
            plt.plot(delivery.pos[0], delivery.pos[1], 'o', color=color, markersize=8)

        # Hesaplanmış yolu çiz
        if route.path:
            xs, ys = zip(*route.path)
            plt.plot(xs, ys, '-', color=color, label=f'Drone {route.drone.id} Rotası')

    # Lejant ekle
    handles, labels = plt.gca().get_legend_handles_labels()
    by_label = dict(zip(labels, handles))
    plt.legend(by_label.values(), by_label.keys())

    # No-fly zone ve teslim edilmemiş noktalar için lejant ekle
    plt.plot([], [], 'r-', label='Uçuş Yasağı Bölgesi')
    plt.plot([], [], 'yo', label='Teslim Edilmemiş')

    plt.title(title)
    plt.xlabel("X (metre)")
    plt.ylabel("Y (metre)")
    plt.grid(True)
    plt.xlim(0, 100)
    plt.ylim(0, 100)
    plt.tight_layout()
    plt.show()


# -------------------------
# 7. Performans Analizi
# -------------------------

def analyze_performance(routes: List[Route], deliveries: List[DeliveryPoint], algorithm_time: float):
    """Algoritmanın performansını analiz eder"""
    # Teslim edilen teslimat sayısı
    delivered_ids = set()
    for route in routes:
        for delivery in route.deliveries:
            delivered_ids.add(delivery.id)

    total_deliveries = len(delivered_ids)
    delivery_percentage = (total_deliveries / len(deliveries)) * 100

    # Enerji tüketimi
    valid_routes = [r for r in routes if r.valid]
    total_energy = sum(route.energy_consumption for route in valid_routes)
    avg_energy = total_energy / len(valid_routes) if valid_routes else 0

    # Toplam mesafe
    total_distance = sum(route.total_distance for route in valid_routes)
    avg_distance = total_distance / len(valid_routes) if valid_routes else 0

    # Sonuçları yazdır
    print("\nPerformans Analizi:")
    print(f"Tamamlanan teslimat sayısı: {total_deliveries}/{len(deliveries)}")
    print(f"Tamamlanan teslimat yüzdesi: {delivery_percentage:.2f}%")
    print(f"Ortalama enerji tüketimi: {avg_energy:.2f} birim")
    print(f"Ortalama mesafe: {avg_distance:.2f} metre")
    print(f"Algoritma çalışma süresi: {algorithm_time:.2f} saniye")

    # Drone bazında sonuçlar
    print("\nDrone Bazında Sonuçlar:")
    for route in routes:
        if route.valid:
            print(f"Drone {route.drone.id}: {len(route.deliveries)} teslimat, "
                  f"{route.total_distance:.2f} metre, {route.energy_consumption:.2f} enerji")

    return {
        "delivery_percentage": delivery_percentage,
        "avg_energy": avg_energy,
        "avg_distance": avg_distance,
        "algorithm_time": algorithm_time,
        "total_deliveries": total_deliveries
    }


# -------------------------
# 8. Zaman Karmaşıklığı Analizi
# -------------------------

def time_complexity_analysis():
    """Algoritmanın zaman karmaşıklığını analiz eder"""
    print("\nZaman Karmaşıklığı Analizi:")
    print("1. A* Algoritması: O(E log V), E = kenar sayısı, V = düğüm sayısı")
    print(
        "2. Genetik Algoritma: O(G * P * D * A), G = nesil sayısı, P = popülasyon boyutu, D = drone sayısı, A = A* çağrı sayısı")
    print("3. CSP Kontrolü: O(D * T), D = drone sayısı, T = teslimat sayısı")
    print("4. Toplam: O(G * P * D * T * E log V)")


# -------------------------
# 9. Ana Fonksiyon
# -------------------------

def main():
    # Rastgele veri oluştur
    random.seed(42)  # Tekrarlanabilirlik için

    # Senaryo 1: 5 drone, 20 teslimat, 2 no-fly zone
    print("Senaryo 1 başlatılıyor...")
    drones1 = generate_random_drones(5)
    deliveries1 = generate_random_deliveries(20)
    noflyzones1 = generate_random_noflyzones(2)

    print(f"Drone sayısı: {len(drones1)}")
    print(f"Teslimat noktası sayısı: {len(deliveries1)}")
    print(f"No-fly zone sayısı: {len(noflyzones1)}")

    # Genetik algoritma ile rota optimizasyonu
    print("Rota optimizasyonu başlatılıyor...")
    best_routes1, algorithm_time1 = genetic_algorithm(
        drones1, deliveries1, noflyzones1, generations=30, population_size=20
    )

    # Performans analizi
    analyze_performance(best_routes1, deliveries1, algorithm_time1)

    # Sonuçları görselleştir
    print("Rotalar görselleştiriliyor...")
    plot_routes(best_routes1, noflyzones1, deliveries1, "Senaryo 1: 5 Drone, 20 Teslimat, 2 No-Fly Zone")

    # Senaryo 2: 10 drone, 50 teslimat, 5 no-fly zone
    print("\nSenaryo 2 başlatılıyor...")
    drones2 = generate_random_drones(10)
    deliveries2 = generate_random_deliveries(50)
    noflyzones2 = generate_random_noflyzones(5)

    print(f"Drone sayısı: {len(drones2)}")
    print(f"Teslimat noktası sayısı: {len(deliveries2)}")
    print(f"No-fly zone sayısı: {len(noflyzones2)}")

    # Genetik algoritma ile rota optimizasyonu
    print("Rota optimizasyonu başlatılıyor...")
    best_routes2, algorithm_time2 = genetic_algorithm(
        drones2, deliveries2, noflyzones2, generations=30, population_size=20
    )

    # Performans analizi
    analyze_performance(best_routes2, deliveries2, algorithm_time2)

    # Sonuçları görselleştir
    print("Rotalar görselleştiriliyor...")
    plot_routes(best_routes2, noflyzones2, deliveries2, "Senaryo 2: 10 Drone, 50 Teslimat, 5 No-Fly Zone")

    # Zaman karmaşıklığı analizi
    time_complexity_analysis()


if __name__ == "__main__":
    main()

# Bu kodu çalıştırmak için:
# 1. matplotlib kütüphanesinin yüklü olduğundan emin olun
# 2. Python 3.6 veya üstü bir sürüm kullanın
# 3. Kodu bir .py dosyasına kaydedip çalıştırın