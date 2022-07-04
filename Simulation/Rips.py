from Controller import Controller
from Visualization import Plot


def main(): 
    controller = Controller('config.yaml', 'sim_008')
    plot = Plot(controller)
    controller.plot = plot
    i = 0
    while not controller.is_full_covered:
        print('Iteration', i)
        controller.run_iter()
        i += 1
    print(f'{len(controller.robots)} robots were used to cover the map.')
    input('Press any key to end.')

if __name__ == '__main__':
    main()
