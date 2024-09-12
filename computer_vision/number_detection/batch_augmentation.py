#creating samples for 0

from PIL import Image
from pathlib import Path
import matplotlib.pyplot as plt
import numpy as np
import sys
import torch
import numpy as np
import torchvision.transforms as transforms
import torchvision.utils
from torchvision.utils import save_image

image_path = 'printed_digit/0/00.jpg'
og_img = Image.open(image_path)

# CONSIDER: consider making these constants? These numbers are present in multiple files?
for d in range(50,151,50):
    img = transforms.RandomRotation(degrees=d)(og_img)
    img.save(f'0/0_rotate{d}.jpg')
