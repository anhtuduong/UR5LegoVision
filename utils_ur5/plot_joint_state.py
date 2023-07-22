
import matplotlib.pyplot as plt
import numpy as np

def plot_graph(points, time_step=0.1):

    # Colors for each line
    colors = ['red', 'green', 'blue', 'orange', 'purple', 'yellow']

    # Plot each line
    for i in range(len(points[0])):
        line_values = [point[i] for point in points]
        plt.plot(line_values, color=colors[i])

    # Set labels and title
    plt.xlabel('Time (index)')
    plt.ylabel('Value')
    plt.title('Graph with Multiple Lines')

    # Show the graph
    plt.show()

    print('Done plotting graph')

if __name__ == '__main__':
    # Example usage
    points = [[1, 6, 2, 3, 5, 1],
            [2, 5, 4, 2, 4, 3],
            [3, 4, 6, 1, 3, 5],
            [4, 3, 4, 2, 4, 3],
            [5, 2, 2, 3, 5, 1],
            [6, 1, 4, 2, 6, 3]]

    plot_graph(points)

