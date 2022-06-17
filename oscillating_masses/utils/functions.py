"""
Helper functions to be used in other scripts
"""

from time import perf_counter as timer

# Execution time evaluator
def timeit(tic, toc):
  return toc - tic

def print_time(tic, toc):
  print(f"{timeit(tic,toc): .3f} sec")

# Example usage
# tic = timer()
# toc = timer()
# print_time(tic, toc)