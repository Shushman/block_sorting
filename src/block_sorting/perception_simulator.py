import numpy, os, prpy
from prpy.perception import PerceptionMethod, PerceptionModule

class PerceptionSimulator(PerceptionModule):

    def __init__(self):

        super(PerceptionSimulator, self).__init__()
                
    def __str__(self):
        return 'PerceptionSimulator'

    @PerceptionMethod
    def DetectTable(self, robot, **kw_args):
        """
        Detect the table and the bin
        """
        from prpy.rave import Disabled
        from prpy.util import ComputeEnabledAABB

        env = robot.GetEnv()

        # Get the pr-ordata package path
        data_dir = prpy.util.FindCatkinResource('pr_ordata', 'data')

        # Place the table in the environment
        table_path = os.path.join(data_dir, 'furniture', 'table.kinbody.xml')
        table = env.ReadKinBodyXMLFile(table_path)
        env.Add(table)

        with env:
            table.GetLink('padding_conference_table').Enable(False)
            table.GetLink('padding_conference_table').SetVisible(False)
        
        table_in_robot = numpy.array([[0., 0., 1., 0.8],
                                      [1., 0., 0., 0.],
                                      [0., 1., 0., 0.],
                                      [0., 0., 0., 1.]])
        table_in_world = numpy.dot(robot.GetTransform(), table_in_robot)
        table.SetTransform(table_in_world)

        return table

    @PerceptionMethod
    def DetectBlocks(self, robot, table, blocks=[], **kw_args):
        """
        Place blocks on the table
        """
        from prpy.rave import Disabled
        from prpy.util import ComputeEnabledAABB

        if len(blocks) == 0:
            # Add all blocks

            # Import the yaml file telling where to place blocks
            block_config_dir = prpy.util.FindCatkinResource('block_sorting', 'config')
            block_config = os.path.join(block_config_dir, 'block_pattern.yaml')
            with open(block_config, 'r') as f:
                import yaml
                block_yaml = yaml.load(f.read())
        
            env = robot.GetEnv()

            # Place blocks in a pattern on the table
            with Disabled(table, padding_only=True):
                table_aabb = ComputeEnabledAABB(table)

            table_height = table_aabb.pos()[2] + table_aabb.extents()[2]
            table_corner = numpy.eye(4)
            table_corner[:3,3] = [table_aabb.pos()[0] - table_aabb.extents()[0],
                                  table_aabb.pos()[1] - table_aabb.extents()[1],
                                  table_height]
            
            for b in block_yaml:
                block = env.ReadKinBodyXMLFile('objects/block.kinbody.xml')
                block_pose = numpy.eye(4)
                block_pose[:2,3] = b['pose']
                block_in_world = numpy.dot(table_corner, block_pose)
                block_in_world[2,3] = table_height
                block.SetTransform(block_in_world)
                block.SetName(b['name'])
                env.Add(block)
                blocks.append(block)

        return blocks

    @PerceptionMethod
    def DetectBins(self, robot, table, **kw_args):
        """
        Place bins on table
        """
        from prpy.rave import Disabled
        from prpy.util import ComputeEnabledAABB
        env = robot.GetEnv()

        # Place blocks in a pattern on the table
        with Disabled(table, padding_only=True):
            table_aabb = ComputeEnabledAABB(table)

        table_height = table_aabb.pos()[2] + table_aabb.extents()[2]
        table_corner = numpy.eye(4)
        table_corner[:3,3] = [table_aabb.pos()[0] - table_aabb.extents()[0],
                              table_aabb.pos()[1] - table_aabb.extents()[1],
                              table_height]
         # Get the pr-ordata package path
        data_dir = prpy.util.FindCatkinResource('pr_ordata', 'data')

        # Import the yaml file telling where to place blocks
        bin_config_dir = prpy.util.FindCatkinResource('block_sorting', 'config')
        bin_config = os.path.join(bin_config_dir, 'bin_metadata.yaml')
        with open(bin_config, 'r') as f:
            import yaml
            bin_yaml = yaml.load(f.read())

        bins = []
        for b in bin_yaml:
            bin_obj = env.ReadKinBodyXMLFile(os.path.join(data_dir, 'objects', b['kinbody']))

            # Set the pose
            bin_pose = numpy.eye(4)
            bin_pose[:2,3] = b['pose']
            bin_in_world = numpy.dot(table_corner, bin_pose)
            bin_in_world[2,3] = table_height
            bin_obj.SetTransform(bin_in_world)

            # Set the color
            for l in bin_obj.GetLinks():
                for g in l.GetGeometries():
                    color = b['color'] + [1.]
                    g.SetDiffuseColor(numpy.array(color))

            # Set the name
            bin_obj.SetName(b['name'])
            
            # Add it
            env.Add(bin_obj)
            bins.append(bin_obj)

        return bins
