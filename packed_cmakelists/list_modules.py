#!/usr/bin/env python
# coding=utf-8

'''To comfort pylint who barks when it can't find a docstring'''

import os
import sys


def generate_line(item):
  '''To comfort pylint who barks when it can't find a docstring'''
  return "add_subdirectory (${PROJECT_SOURCE_DIR}/"+item+" "+item+")"

def traverse_for_cmakelists(root_dir):
  '''To comfort pylint who barks when it can't find a docstring'''
  sub_items = os.listdir(root_dir)
  sub_dirs = []
  for i in sub_items:
    if os.path.isdir(root_dir + os.sep + i):
      sub_dirs.append(i)

  sub_dirs.sort()
  results = []
  for sub_d in sub_dirs:
    if os.path.exists(root_dir + os.sep + sub_d + os.sep + "CMakeLists.txt"):
      results.append(sub_d)
    else:
      sub_results = traverse_for_cmakelists(root_dir + os.sep + sub_d)
      for sub_r in sub_results:
        results.append(sub_d + os.sep + sub_r)
  return results

def main(argv):
  '''To comfort pylint who barks when it can't find a docstring'''
  results_all = traverse_for_cmakelists(argv[1])
  lines = []
  for result in results_all:
    lines.append(generate_line(result))
  for line in lines:
    print line

if __name__ == '__main__':
  main(sys.argv)
