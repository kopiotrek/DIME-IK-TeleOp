import numpy as np

import matplotlib.pyplot as plt
import signal
import sys

class PlotMediapipeHand(object):
    def __init__(self, calibration_data = None):
        self.fig = plt.figure()
        self.calibration_data = calibration_data
        signal.signal(signal.SIGTERM, self.handle_exit)
        signal.signal(signal.SIGINT, self.handle_exit)

    def plot_hand(self, X, Y):
        X, Y = np.array(X).flatten(), np.array(Y).flatten()

        # Drawing connections fromn the wrist
        plt.plot([X[0], X[1]], [Y[0], Y[1]])
        plt.plot([X[0], X[2]], [Y[0], Y[2]])
        plt.plot([X[0], X[3]], [Y[0], Y[3]])
        plt.plot([X[0], X[4]], [Y[0], Y[4]])
        plt.plot([X[0], X[5]], [Y[0], Y[5]])

        # Drawing knuckle to knuckle connections
        plt.plot([X[2], X[3]], [Y[2], Y[3]])
        plt.plot([X[3], X[4]], [Y[3], Y[4]])
        plt.plot([X[4], X[5]], [Y[4], Y[5]])

        # Drawing knuckle to finger connections
        plt.plot([X[1], X[6]], [Y[1], Y[6]])
        plt.plot([X[2], X[7]], [Y[2], Y[7]])
        plt.plot([X[3], X[8]], [Y[3], Y[8]])
        plt.plot([X[4], X[9]], [Y[4], Y[9]])
        plt.plot([X[5], X[10]], [Y[5], Y[10]])

    def plot_bounds(self):
        if self.calibration_data is not None:
            thumb_bounds = self.calibration_data[4:]

            # Plotting the thumb bounds
            plt.plot([thumb_bounds[0][0], thumb_bounds[1][0]], [thumb_bounds[0][1], thumb_bounds[1][1]])
            plt.plot([thumb_bounds[1][0], thumb_bounds[2][0]], [thumb_bounds[1][1], thumb_bounds[2][1]])
            plt.plot([thumb_bounds[2][0], thumb_bounds[3][0]], [thumb_bounds[2][1], thumb_bounds[3][1]])
            plt.plot([thumb_bounds[3][0], thumb_bounds[0][0]], [thumb_bounds[3][1], thumb_bounds[0][1]])

            # TODO
            # Plot the bounds for other fingers

    def draw(self, X, Y):
        # Plotting the 2D points and lines in the graph
        # Plotting the Joint coordinates
        plt.plot(X, Y, 'ro')

        self.plot_bounds()        

        # Plotting the lines to visualize the hand
        self.plot_hand(X, Y)
        
        # Setting the visualization axis
        plt.axis([-1, 1.3, -0.2, 2.25])

        plt.draw()

        # Resetting and Pausing the 3D plot
        self.fig.canvas.flush_events()
        plt.pause(0.01)
        plt.cla()

    def handle_exit(self, signum, frame):
        plt.close('all')
        print('Exiting Visualizer...')
        sys.exit(0)