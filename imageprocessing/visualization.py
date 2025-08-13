import matplotlib.pyplot as plt
from matplotlib.patches import Patch
from metadata import _CLUSTER_NAMES


def visualize_clusters(clusters, title="Кластеризация изображения", point_size=20, background_color='white'):
    plt.figure(figsize=(10, 8), facecolor=background_color)
    plt.title(title, fontsize=16)

    colors = ['red', 'green', 'blue', 'purple']
    legend_elements = []

    for i, cluster in enumerate(clusters):
        if cluster:
            x = [p[0] for p in cluster]
            y = [p[1] for p in cluster]
            plt.scatter(x, y, s=point_size, color=colors[i], alpha=0.7)
            legend_elements.append(Patch(color=colors[i], label=_CLUSTER_NAMES[i]))

    plt.axis('equal')
    plt.grid(alpha=0.2)
    plt.legend(handles=legend_elements, loc='best')
    plt.tight_layout()

    plt.savefig('clusters_visualization.png', dpi=150, facecolor=background_color)
    plt.show()
