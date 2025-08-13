"""
Clasterization of an image
"""

import math
from collections import deque
from heapq import heapify, heappop, heappush
from scipy.spatial import Delaunay
import numpy as np


def build_mst_from_delaunay(points: tuple) -> list:
    if len(points) < 2:
        return []

    # Нахождение корневого элемента множества
    def find(x: int) -> int:
        if parent[x] != x:
            parent[x] = find(parent[x])

        return parent[x]

    # Объединение множеств
    def union(x: int, y: int) -> bool:
        rx, ry = find(x), find(y)

        if rx == ry:
            return False
        if rank[rx] < rank[ry]:
            parent[rx] = ry
        elif rank[rx] > rank[ry]:
            parent[ry] = rx
        else:
            parent[ry] = rx
            rank[rx] += 1

        return True

    points_array = np.array(points)
    tri = Delaunay(points_array)
    edges_set = set()
    for simplex in tri.simplices:
        for i in range(3):
            a, b = simplex[i], simplex[(i + 1) % 3]
            edges_set.add((min(a, b), max(a, b)))

    edges = []
    n = len(points)
    for a, b in edges_set:
        dist = math.dist(points[a], points[b])
        edges.append((dist, a, b))

    edges.sort(key=lambda x: x[0])

    parent = list(range(n))
    rank = [0] * n
    mst_edges = []

    for d, a, b in edges:
        if union(a, b):
            mst_edges.append((a, b))
            if len(mst_edges) == n - 1:
                break

    return mst_edges


def cluster_points(points: tuple) -> tuple:
    n = len(points)
    if n <= 4:
        clusters = []
        for i in range(4):
            clusters.append(tuple(points[i] for i in [j for j in range(n) if j % 4 == i]))
        while len(clusters) < 4:
            clusters.append(())

        return tuple(clusters)

    mst_edges = build_mst_from_delaunay(points)
    if not mst_edges:
        return (), (), (), ()

    graph = [[] for _ in range(n)]
    for u, v in mst_edges:
        graph[u].append(v)
        graph[v].append(u)

    parent_full = [-1] * n
    children_full = [[] for _ in range(n)]
    size_full = [1] * n
    queue = deque([0])
    visited = [False] * n
    visited[0] = True
    order = []
    while queue:
        u = queue.popleft()
        order.append(u)
        for v in graph[u]:
            if not visited[v]:
                visited[v] = True
                parent_full[v] = u
                children_full[u].append(v)
                queue.append(v)

    for u in order[::-1]:
        for v in children_full[u]:
            size_full[u] += size_full[v]

    max_heap = [(-n, 0)]
    heapify(max_heap)
    roots = {0}

    for _ in range(3):
        if not max_heap:
            break

        neg_size, root = heappop(max_heap)
        size_tree = -neg_size
        if size_tree <= 1:
            heappush(max_heap, (neg_size, root))
            continue

        stack = [root]
        best_diff = float('inf')
        best_edge = None
        best_size = 0
        while stack:
            u = stack.pop()
            for v in children_full[u]:
                stack.append(v)
                current_diff = abs(size_full[v] - n / 4.0)
                if current_diff < best_diff:
                    best_diff = current_diff
                    best_edge = (u, v)
                    best_size = size_full[v]

        if best_edge is None:
            heappush(max_heap, (neg_size, root))
            continue

        u, v = best_edge
        children_full[u].remove(v)
        parent_full[v] = -1
        current = u
        while current != -1:
            size_full[current] -= best_size
            current = parent_full[current]

        heappush(max_heap, (-(size_tree - best_size), root))
        heappush(max_heap, (-best_size, v))
        roots.add(v)

    clusters = []
    for root in roots:
        cluster_vertices = []
        stack = [root]
        while stack:
            u = stack.pop()
            cluster_vertices.append(u)
            for v in children_full[u]:
                stack.append(v)

        clusters.append(tuple(points[i] for i in cluster_vertices))

    while len(clusters) < 4:
        clusters.append(())

    return tuple(clusters)
