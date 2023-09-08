
# Printed Numerical Digits Image Dataset.

## Version : v0.1 Beta

Contains around 3000 images of digitally printed numeric dataset.

Each image is of dimension `28x28` and is grayscale. This dataset was purposely created for sudoku digits classification hence it shows blank image for 0 (zeros).

This dataset originally had 177 images, afterwards augmentation was performed.


<img src="https://github.com/kaydee0502/printed-digits-dataset/blob/master/assets/4/0_0_179.jpeg" width="100" height="100" align = "left"/>
<img src="https://github.com/kaydee0502/printed-digits-dataset/blob/master/assets/2/0_0_2505.jpeg" width="100" height="100" align="middle"/>
<img src="https://github.com/kaydee0502/printed-digits-dataset/blob/master/assets/9/0_0_9281.jpeg" width="100" height="100" align="left"/>
<img src="https://github.com/kaydee0502/printed-digits-dataset/blob/master/assets/5/0_0_5751.jpeg" width="100" height="100" align="middle"/>

## Repository Structure:

+ All images are in [assets](https://github.com/kaydee0502/printed-digits-dataset/tree/master/assets) forlder with their respective names, example: folder [assets/1](https://github.com/kaydee0502/printed-digits-dataset/tree/master/assets/1) contains all images with label `1`.

+ Folder [model](https://github.com/kaydee0502/printed-digits-dataset/tree/master/model) contains a pretrained CNN model trained on this data (96% accuracy on 10 epochs with batch size 10).

+ File [loader.py](https://github.com/kaydee0502/printed-digits-dataset/blob/master/loader.py) is a helper file to convert sudoku images into cells and save them as desired output in respective assets folder, Note that this script uses a global counter variable stored in counter.pickle.


## Contribute:
We still required more data, apart from that images of digit `0` is needed, Pull Requests are most welcome and appreciated..

## License:

> MIT License

Copyright (c) 2021 Kshitij Dhama

Permission is hereby granted, free of charge, to any person obtaining a copy
of this software and associated documentation files (the "Software"), to deal
in the Software without restriction, including without limitation the rights
to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
copies of the Software, and to permit persons to whom the Software is
furnished to do so, subject to the following conditions:

The above copyright notice and this permission notice shall be included in all
copies or substantial portions of the Software.

THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
SOFTWARE.
