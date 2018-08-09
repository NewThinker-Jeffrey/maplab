#! /usr/bin/env python3

'''To comfort pylint who barks when it can't find a docstring'''

import os
import sys

def create_multimap_optimiz_yaml(
    folderlist, merged_map_key,
    paramset='', outputdir='',
    vimap_to_anchor_to='', summary_map_to_anchor_to=''):
  '''To comfort pylint who barks when it can't find a docstring'''

  if len(outputdir) == 0:
    outputdir = os.getcwd()
  script_path = outputdir + os.sep + 'multi_map_optimization_scripts.yaml'
  script_file = open(script_path, 'w')
  script_file.write('# Merge multi vimaps and optimize the merged map.\n')
  script_file.write('vi_map_folder_paths:\n')
  script_file.write('  - ___place_holder___\n')
  script_file.write('commands:\n')
  foldercount = 0
  mapkey_prefix = "vimap"

  for folder in folderlist:
    mapkey = mapkey_prefix + '_' + str(foldercount)
    loadmsg = '  - load --map_key {0} --map_folder={1}\n'.format(mapkey, folder)
    script_file.write(loadmsg)
    foldercount += 1

  script_file.write('  - select --map_key {0}_0\n'.format(mapkey_prefix))
  script_file.write('# Create a map including all missions\n')
  script_file.write('  - join_all_maps --target_map_key ' +
                    merged_map_key + '\n')
  script_file.write('# Visualize all missions with different colors.\n')
  script_file.write('  - v --vis_color_by_mission\n')
  script_file.write('# Set the first base frame to "known"\n')
  script_file.write('  - sbk {}\n'.format(paramset))
  script_file.write('# Anchor all missions.\n')
  script_file.write('  - aam {}\n'.format(paramset))
  script_file.write('# Visualize all missions with different colors.\n')
  script_file.write('  - v --vis_color_by_mission\n')
  script_file.write('# Only keep keyframes in the map.\n')
  script_file.write('  - kfh --kf_distance_threshold_m 0.2\n')
  script_file.write('  - relax {}\n'.format(paramset))
  script_file.write('# Loop close the map\n')
  script_file.write('  - lc {0}\n'.format(paramset))
  script_file.write('# Bundle adjustment\n')
  script_file.write('  - optvi {}\n'.format(paramset))
  script_file.write('# Repeat lc and optvi to further improve ' +
                    'the quality of the map\n')
  script_file.write('  - lc {0}\n'.format(paramset))
  script_file.write('# Bundle adjustment\n')
  script_file.write('  - optvi {}\n'.format(paramset))

  if vimap_to_anchor_to != '':
    script_file.write('# Reanchor to another vimap\n')
    script_file.write('  - racl --another_vimap {}\n'
                      .format(vimap_to_anchor_to))
  elif summary_map_to_anchor_to != '':
    script_file.write('# Reanchor to another summary map\n')
    script_file.write('  - racl --another_summary_map {}\n'
                      .format(summary_map_to_anchor_to))

  output_vimap_dir = outputdir+os.sep+"vimap"
  output_csv_dir = outputdir+os.sep+"csv"
  output_summary_map_path = outputdir+os.sep+"summary_map"

  cmd_vimap = '  - save --map_folder {0}\n'.format(output_vimap_dir)
  script_file.write(cmd_vimap)

  cmd_summary_map = '  - generate_summary_map_and_save_to_disk ' + \
                    '--summary_map_save_path ' + output_summary_map_path + \
                    ' ' + paramset + '\n'
  script_file.write(cmd_summary_map)

  # export csv files
  cmd_csv = '  - csv_export --csv_export_path {0}\n'.format(output_csv_dir)
  script_file.write(cmd_csv)

  script_file.close()
  return script_path


def merge_maps(vimap_folder_list, merged_map_key,
               paramset='', outputdir='', vimap_to_anchor_to='',
               summary_map_to_anchor_to=''):
  '''To comfort pylint who barks when it can't find a docstring'''
  batch_control_file = create_multimap_optimiz_yaml(
      vimap_folder_list, merged_map_key,
      paramset=paramset, outputdir=outputdir,
      vimap_to_anchor_to=vimap_to_anchor_to,
      summary_map_to_anchor_to=summary_map_to_anchor_to)
  my_loc = os.path.abspath(os.path.split(__file__)[0])
  batch_script = my_loc + os.sep + \
                 "../../applications/maplab-console/scripts/run_maplab_batch.sh"
  cmdline = batch_script+' '+batch_control_file
  print 'merge vimaps with command: '
  print cmdline
  return os.system(cmdline)


if __name__ == '__main__':
  print sys.argv[0]
