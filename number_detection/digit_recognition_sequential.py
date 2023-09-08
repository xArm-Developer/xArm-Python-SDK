import numpy as np
import torch
import torchvision
import matplotlib.pyplot as plt
from time import time
from torchvision import datasets, transforms
from torch import nn, optim

# get data
transform = transforms.Compose([transforms.ToTensor(),
                              transforms.Normalize((0.5,), (0.5,)),
                              ])
# transform data
# split into training and test sets
trainset = datasets.MNIST('data', download=True, train=True, transform=transform)
valset = datasets.MNIST('data', download=True, train=False, transform=transform)
trainloader = torch.utils.data.DataLoader(trainset, batch_size=64, shuffle=True)
valloader = torch.utils.data.DataLoader(valset, batch_size=64, shuffle=True)

# get data if already exists?

#build neural net
input_size = 784 # 28 * 28 input image size
hidden_sizes = [128, 64]
output_size = 10

model = nn.Sequential(
    nn.Linear(input_size, hidden_sizes[0]),
    nn.ReLU(),
    nn.Linear(hidden_sizes[0], hidden_sizes[1]),
    nn.ReLU(),
    nn.Linear(hidden_sizes[1], output_size),
    nn.LogSoftmax(dim=1)
)

# define loss
criterion = nn.CrossEntropyLoss()

# adjust weights

#optimizer = optim.SGD(model.parameters(), lr=0.001, momentum=0.9)
optimizer = optim.Adam(model.parameters(), lr=0.001)
time0 = time()
epochs = 100

for e in range(epochs):
    running_loss = 0
    for images, labels in trainloader:
        #flatten data into 1D vector
        images = images.view(images.shape[0], -1)

        #Training pass
        optimizer.zero_grad()

        output = model(images)
        loss = criterion(output, labels)

        #backpass
        loss.backward()

        #optimize weights
        optimizer.step()

        running_loss += loss.item()
    
    else:
        print("Epoch {} - Training loss: {}".format(e,
        running_loss/len(trainloader)))
    print("\nTraining Time (in minutes) =",(time()-time0)/60)

#save model
torch.save(model, 'mnist1-crossentropy-adam.pt')

# Test model
with torch.no_grad():
    n_correct = 0
    n_samples = 0
    for images, labels in valloader:
        for i in range(len(labels)):
            img = images[i].view(1, 784)
            with torch.no_grad():
                logps = model(img)

            ps = torch.exp(logps)
            probab = list(ps.numpy()[0])
            pred_label = probab.index(max(probab))
            true_label = labels.numpy()[i]
            if(true_label == pred_label):
                n_correct += 1
            n_samples += 1

    print("Number Of Images Tested =", n_samples)
    print("\nModel Accuracy =", (n_correct/n_samples))