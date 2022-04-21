
# -*- coding: utf-8 -*-
"""
Created on Thu Apr 21 16:44:57 2022

@author: skotw
"""
import copy
import math
import random
import sys

import matplotlib.pyplot as plt
import numpy as np

from dataset_processing.soda_dataloader import SODA

# Load dataset
soda = SODA()
x = soda.coastal_lats
y = soda.coastal_lons
plt.scatter(x,y)

