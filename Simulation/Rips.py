from Controller import Controller
from Visualization import Plot


def main(): 
    controller = Controller('config.yaml', 'sim_004')
    plot = Plot(controller)
    controller.plot = plot
    while not controller.is_full_covered:
        controller.run_iter()

    input('Press any key to end.')

if __name__ == '__main__':
    main()
