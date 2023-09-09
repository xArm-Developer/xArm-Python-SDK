import torch
import torch.nn as nn
import torch.optim as optim
from PIL import Image
from pathlib import Path
import numpy as np
import torchvision.transforms as transforms
import torchvision.utils
from torchvision.utils import save_image
from digit_recognition import Net

# get input image
#image_path = 'opencv_frame_0.png'
#image_path = 'printed_digit/train/3/0_0_7.jpeg'
image_path = 'image_processing_cv/dig_1.png'
# processing -> resize and turn B&W (white on black)

og_img = Image.open(image_path)
img = transforms.Compose([transforms.ToTensor(),
                            transforms.Resize((28, 28)),
                            #transforms.Grayscale(1),
                            transforms.Normalize((0.5,), (0.5,))])(og_img)

#torchvision.transforms.functional.invert(img)
input = img.reshape(-1, 784) # grayscale color channel transforms.Grayscale

# load model
model_dir = 'random_gaussian_1.pt'
state = torch.load(model_dir)
model = Net()
try:
    state.eval()
except AttributeError as error:
    print(error)

model.load_state_dict(state['model_state_dict'])
model.eval()

# make prediction
with torch.no_grad():
    output = model.predict(input)

print(torch.argmax(output).item())