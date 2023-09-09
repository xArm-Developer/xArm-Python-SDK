import numpy as np
import torch
import torchvision
import matplotlib.pyplot as plt
from time import time
from torchvision import datasets, transforms
from torch import nn, optim

'''
    simple digit recognition neural net trained on MNIST
'''
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

class Net(nn.Module):
    def __init__(self):
        super(Net, self).__init__()
        self.model = nn.Sequential(
            nn.Linear(input_size, hidden_sizes[0]),
            nn.ReLU(),
            nn.Linear(hidden_sizes[0], hidden_sizes[1]),
            nn.ReLU()
        )
        self.last_layer = nn.Linear(hidden_sizes[1], output_size)
        #self.soft_max = nn.LogSoftmax(dim=1)

    def forward(self, x):
        logits = self.model(x)
        logits = self.last_layer(logits)
        return logits

    def predict(self, x):
        logits = self.model(x)
        logits = self.last_layer(logits)
        softmax = torch.nn.Softmax(dim=1)
        return softmax(logits)


def train_model():
    # define loss
    criterion = nn.CrossEntropyLoss()

    # adjust weights
    network = Net()
    optimizer = optim.SGD(network.parameters(), lr=0.001, momentum=0.9)
    #optimizer = optim.Adam(network.parameters(), lr=0.001)
    time0 = time()
    epochs = 100

    for e in range(epochs):
        running_loss = 0
        for images, labels in trainloader:
            #flatten data into 1D vector
            images = images.view(images.shape[0], -1)
            

            #Training pass
            optimizer.zero_grad()

            
            output = network(images)
            # output = network.last_layer(output)
            # output = network.soft_max(output)
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
    torch.save(network, 'mnist1-ce-sgd-3.pt')

    # Test model
    with torch.no_grad():
        n_correct = 0
        n_samples = 0
        for images, labels in valloader:
            for i in range(len(labels)):
                img = images[i].view(1, 784)
                with torch.no_grad():
                    logps = network.predict(img)

                ps = torch.exp(logps)
                probab = list(ps.numpy()[0])
                pred_label = probab.index(max(probab))
                true_label = labels.numpy()[i]
                if(true_label == pred_label):
                    n_correct += 1
                n_samples += 1

        print("Number Of Images Tested =", n_samples)
        print("\nModel Accuracy =", (n_correct/n_samples))

if __name__ == '__main__':

    train_model()