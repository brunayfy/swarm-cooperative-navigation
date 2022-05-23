from Controller import Controller
from Visualization import Plot


def main(): 
    controller = Controller('config.yaml', 'sim_001')
    plot = Plot(controller)
    controller.plot = plot

    while not controller.is_full_covered():
        controller.run_iter()
        plot.update_plot()


if __name__ == '__main__':
    main()
