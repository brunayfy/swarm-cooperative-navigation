"""
Simulation Settings
"""
import DotBot
import Floorplan
import Orchestrator
import SimEngine
import SimUI
import Wireless


class SimSettings:
    def __init__(self, numDotBots, floorplanDrawing, initialPosition=(0, 0), robot_radius=1, show_radius=False,
                 show_rips_complex=False):
        self.numDotBots = numDotBots
        self.floorplanDrawing = floorplanDrawing  # 1m per character
        self.initialPosition = initialPosition
        self.robot_radius = robot_radius
        self.show_radius = show_radius
        self.show_rips_complex = show_rips_complex


# ============================ defines =========================================

SIMSETTINGS = [
    SimSettings(
        numDotBots=10,
        floorplanDrawing='''
            ..................
            ..................
            ............##....
            ............##....
            ..................
            ..................
            ''',
        initialPosition=(18, 4.5),
        robot_radius=100,
        show_radius=True
    )
]


# ============================ helpers =========================================

def oneSim(simSetting):
    """
    Run a single simulation.
    """

    # create the wireless communication
    wireless = Wireless.Wireless()

    # create the SimEngine
    simEngine = SimEngine.SimEngine()

    # create the floorplan
    floorplan = Floorplan.Floorplan(simSetting.floorplanDrawing)

    # create the DotBots
    dotBots = []
    for dotBotId in range(simSetting.numDotBots):
        dotBots += [DotBot.DotBot(dotBotId, floorplan)]

    # drop the DotBots on the floorplan at their initial position
    (x, y) = simSetting.initialPosition
    for dotBot in dotBots:
        dotBot.setInitialPosition(x, y)

    # create the orchestrator
    orchestrator = Orchestrator.Orchestrator([simSetting.initialPosition] * len(dotBots), floorplan)

    # indicate the elements to the singletons
    wireless.indicateElements(dotBots, orchestrator)

    # start the UI (call last)
    SimUI.SimUI(floorplan, dotBots, orchestrator, simSetting)

    # schedule the first event
    simEngine.schedule(0, orchestrator.startExploration)

    input('Press Enter to close simulation.')


# ============================ main ============================================

def main():
    for simSetting in SIMSETTINGS:
        oneSim(simSetting)


if __name__ == '__main__':
    main()
