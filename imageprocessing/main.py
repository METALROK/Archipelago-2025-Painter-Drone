"""
main file
"""

from metadata import _IMAGE, _CLUSTER_NAMES
from encoder import get_line_points
from clasterization import cluster_points
from visualization import visualize_clusters


points = get_line_points(_IMAGE)
clusters = cluster_points(points)
for i, cluster in enumerate(clusters):
    print(f"{_CLUSTER_NAMES[i]}: {len(cluster)} points")

visualize_clusters(
    clusters,
    title="Кластеризация изображения на 4 непрерывные области",
    point_size=15,
    background_color='white'
)
