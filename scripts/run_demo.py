#!/usr/bin/env python
import argparse, herbpy, logging, numpy, os, prpy, random
from prpy.planning import PlanningError

logger = logging.getLogger('block_sorting')
logger.setLevel(logging.INFO)
ch = logging.StreamHandler()
logger.addHandler(ch)

if __name__ == '__main__':
    
    parser = argparse.ArgumentParser(description="Detect and sort blocks on a table")
    parser.add_argument("-r", "--nosim", dest="nosim", action="store_true", default=False,
                      help="Run on the real robot rather than in simulation mode.")
    parser.add_argument("-v", "--viewer", dest="viewer", default=None,
                        help="The viewer to attach")
    parser.add_argument("-a", "--arm", dest="arm", default='right',
                        help="The arm to grab the blocks with")

    args = parser.parse_args()
    sim = not args.nosim

    herbpy_args = {'left_arm_sim': sim,
                   'left_hand_sim': sim,
                   'right_arm_sim': sim,
                   'right_hand_sim': sim,
                   'head_sim': sim,
                   'segway_sim': True,
                   'perception_sim': sim}
    env, robot = herbpy.initialize(**herbpy_args)
    if args.viewer:
        env.SetViewer(args.viewer) # 14.04 weirdness

    # Make the manipulator active
    manip = robot.GetManipulator(args.arm)
    robot.SetActiveManipulator(manip)

    # Detect static objects in the environment
    if sim:
        from block_sorting.perception_simulator import PerceptionSimulator
        robot.detector = PerceptionSimulator()
    robot.DetectObjects()

    # Grab the table
    table = env.GetKinBody('conference_table')
    block_bin = env.GetKinBody('wicker_tray')

    running = True
    while running:
        logger.info('Redecting objects')
        blocks = robot.DetectBlocks(table)

        idx = random.randint(0, len(blocks) - 1)

        # TODO: Draw a circle around the block currently being grasped
        block = blocks[idx]
        
        logger.info('Grabbing block %s' % block.GetName())
        raw_input('Press enter to continue')
        try:
            robot.GrabBlock(block, table, manip=manip)
            robot.PlaceBlock(block, block_bin, manip=manip)
        except PlanningError, e:
            logger.error('Failed to grab block')
            raw_input('Press any key to continue')
            
        try:
            with prpy.rave.Disabled(block_bin, padding_only=True):
                with prpy.rave.Disabled(block):
                    manip.PlanToNamedConfiguration('home')
        except PlanningError, e:
#            robot.Say("I am stuck. Can you help me?")
            if sim:
                indices, config = robot.configurations.get_configuration('home')
                robot.SetDOFValues(config, dofindices=indices)
            else:
                raw_input('Press enter to continue')
#            robot.Say("Thank you.")

    import IPython; IPython.embed()


        
