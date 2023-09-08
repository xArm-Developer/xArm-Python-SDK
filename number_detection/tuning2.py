import numpy as np
import torch
import torchvision
import matplotlib.pyplot as plt
from time import time
from torchvision import datasets, transforms
from torch import nn, optim
import os
import copy
from torch.optim import lr_scheduler
from digit_recognition import Net

data_dir = 'printed_digit'

#load dataset
# dataset = []
# for i in range(10):
#     for d in os.listdir("printed_digit/{}".format(i)):
#         t_img = cv2.imread("printed_digit/{}".format(i)+"/"+d)
#         t_img = cv2.cvtColor(t_img,cv2.COLOR_BGR2GRAY)
#         dataset.append((t_img, i))

'''
    Add randomized gaussian noise to images in the training and test set
'''
class AddGaussianNoise(object):
    def __init__(self, mean=0., std=1.):
        self.std = std
        self.mean = mean
        
    def __call__(self, tensor):
        return tensor + torch.randn(tensor.size()) * self.std + self.mean
    
    def __repr__(self):
        return self.__class__.__name__ + '(mean={0}, std={1})'.format(self.mean, self.std)
    
kernel_size = (5, 11)
sigma = (0.1, 0.2)

data_transforms = {
    'train': transforms.Compose([transforms.ToTensor(),
                                transforms.Normalize((0.5,), (0.5,)),
                                transforms.RandomApply([AddGaussianNoise(0., 1.)]),
                                transforms.GaussianBlur(kernel_size=kernel_size, sigma=sigma)
                                ]),
                        
    'val': transforms.Compose([transforms.ToTensor(),
                            transforms.Normalize((0.5,), (0.5,)),
                            transforms.RandomApply([AddGaussianNoise(0., 1.)]),
                            transforms.GaussianBlur(kernel_size=kernel_size, sigma=sigma)
                            ])
}

# train_size = int(0.8 * len(dataset))
# test_size = len(dataset) - train_size
# train, val = torch.utils.data.random_split(dataset, [train_size, test_size])

image_datasets = {x: datasets.ImageFolder(os.path.join(data_dir, x),
                                          data_transforms[x])
                  for x in ['train', 'val']}
dataloaders = {x: torch.utils.data.DataLoader(image_datasets[x], batch_size=4,
                                             shuffle=True, num_workers=4)
              for x in ['train', 'val']}

dataset_sizes = {x: len(image_datasets[x]) for x in ['train', 'val']}
#class_names = image_datasets['train'].classes

device = torch.device("cuda:0" if torch.cuda.is_available() else "cpu")

def train_model(model, criterion, optimizer, scheduler, num_epochs=100):
    since = time()

    best_model_wts = copy.deepcopy(model.state_dict())
    best_acc = 0.0

    for epoch in range(num_epochs):
        print(f'Epoch {epoch}/{num_epochs - 1}')
        print('-' * 10)

        # Each epoch has a training and validation phase
        for phase in ['train', 'val']:
            if phase == 'train':
                model.train()  # Set model to training mode
            else:
                model.eval()   # Set model to evaluate mode

            running_loss = 0.0
            running_corrects = 0

            # Iterate over data.
            for inputs, labels in dataloaders[phase]:
                inputs = inputs.to(device)
                labels = labels.to(device)

                # zero the parameter gradients
                optimizer.zero_grad()
                # forward
                # track history if only in train
                with torch.set_grad_enabled(phase == 'train'):
                    inputs = inputs[:, 0, ...].reshape(-1, 784) 
                    if (phase == 'train'):
                        outputs = model(inputs)
                    else:
                        outputs = model.predict(inputs)
                    _, preds = torch.max(outputs, 1)
                    loss = criterion(outputs, labels)

                    # backward + optimize only if in training phase
                    if phase == 'train':
                        loss.backward()
                        optimizer.step()

                # statistics
                running_loss += loss.item() * inputs.size(0)
                running_corrects += torch.sum(preds == labels.data)
            if phase == 'train':
                scheduler.step()

            epoch_loss = running_loss / dataset_sizes[phase]
            epoch_acc = running_corrects.double() / dataset_sizes[phase]

            print(f'{phase} Loss: {epoch_loss:.4f} Acc: {epoch_acc:.4f}')

            # deep copy the model
            if phase == 'val' and epoch_acc > best_acc:
                best_acc = epoch_acc
                best_model_wts = copy.deepcopy(model.state_dict())
                torch.save({
                    'epoch': epoch + 1,
                    'model_state_dict': model.state_dict(),
                    'optimizer_state_dict': optimizer.state_dict(),
                    'loss': criterion
                }
                , 'random_gaussian_1.pt')

        print()

    time_elapsed = time() - since
    print(f'Training complete in {time_elapsed // 60:.0f}m {time_elapsed % 60:.0f}s')
    print(f'Best val Acc: {best_acc:4f}')

    # load best model weights
    model.load_state_dict(best_model_wts)
    return model

model_dir = 'models/transfer-ce-sgd-4.pt'

model = Net()
state = torch.load(model_dir)
model.load_state_dict(state['model_state_dict'])



num_ftrs = 64 # how to not hardcode this? what is it

# Here the size of each output sample is set to 2.
# Alternatively, it can be generalized to nn.Linear(num_ftrs, len(class_names)).
model.last_layer = nn.Linear(num_ftrs, 10)

model = model.to(device)

criterion = nn.CrossEntropyLoss()

# Observe that all parameters are being optimized
optimizer_ft = optim.SGD(model.parameters(), lr=0.001, momentum=0.9)

# Decay LR by a factor of 0.1 every 7 epochs
exp_lr_scheduler = lr_scheduler.StepLR(optimizer_ft, step_size=7, gamma=0.1)

if __name__ == '__main__':
    model = train_model(model, criterion, optimizer_ft, exp_lr_scheduler, num_epochs=100)
