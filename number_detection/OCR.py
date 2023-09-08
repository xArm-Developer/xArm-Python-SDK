import torch
from PIL import Image
import numpy as np
import torchvision.transforms as transforms
from digit_recognition import Net
import cv2
import numpy as np

class OCR():
    VERT_PADDING = 11
    HZ_PADDING = 3
    digit_imgs = []
    volume = 0.0

    def __init__(self, image) -> None:
        self.image = image
        self.net = Net()

    '''
        public method to check if a number exists
    '''
    
    def get_number(self):
        digit_imgs = self.isolate_digits()
        if (digit_imgs == None):
            return "Invalid Volume", 0
        place = 10
        for digit in digit_imgs:
            self.volume += self.predict(digit) * place
            place /= 10
        return "The number is: ", self.volume
        

    '''
        takes the image and creates bounding boxes around each digit
    '''
    def isolate_digits(self):
        # resize image
        # load the example image
        image = self.image
        crop_img = image[350:600, 200:450]

        # pre-process the image by resizing it, converting it to
        # graycale, blurring it, and computing an edge map
        gray = cv2.cvtColor(crop_img, cv2.COLOR_BGR2GRAY)
        # Applying Gaussian blurring with a 5×5 kernel to reduce high-frequency noise

        thresh1 = cv2.threshold(gray, 150, 255, cv2.THRESH_BINARY | cv2.THRESH_OTSU)[1]

        kernel1 = cv2.getStructuringElement(cv2.MORPH_RECT, (2, 2))
        thresh1 = cv2.morphologyEx(thresh1, cv2.MORPH_OPEN, kernel1)

        blurred = cv2.GaussianBlur(thresh1, (5, 5), 0)
        edged = cv2.Canny(blurred, 50, 200, 255)


        # get black box contour: 
        # find contours in the edge map, then sort them by their
        # size in descending order
        cnts, _ = cv2.findContours(edged, cv2.RETR_EXTERNAL, cv2.CHAIN_APPROX_SIMPLE)
        cnts = sorted(cnts, key=lambda x: cv2.contourArea(x), reverse=True)

        x,y,w,h= cv2.boundingRect(cnts[0])
        cropped_img=crop_img[y:y+h, x:x+w]
        color = cv2.cvtColor(cropped_img, cv2.COLOR_BGR2GRAY)
        print(color.shape)

        # do binary color transform
        # threshold the warped image, then apply a series of morphological
        # operations to cleanup the thresholded image
        thresh = cv2.threshold(color, 150, 255, cv2.THRESH_BINARY | cv2.THRESH_OTSU)[1]
        kernel = cv2.getStructuringElement(cv2.MORPH_ELLIPSE, (2, 2))
        thresh = cv2.morphologyEx(thresh, cv2.MORPH_OPEN, kernel)

        # find contours in the thresholded image, then initialize the
        # digit contours lists

        matrix = np.asarray(thresh[:, 0:170])
        num_rows, num_cols = matrix.shape
        block_tuples = []
        blobs = []
        l_cnt = 0
        r_cnt = 0

        row = int(num_rows / 2)
        flag = False
        # get black regions
        for c in range(matrix.shape[1]):
            if (matrix[row, c] == 0):
                if not flag:
                    l_cnt = r_cnt
                    flag = not flag
            else:
                if flag:
                    block_tuples.append(l_cnt)
                    blobs.append(r_cnt - l_cnt)
                    flag = not flag
                
            r_cnt = r_cnt + 1
        
        if (len(blobs) < 3):
            return None
        
        top_three = sorted(zip(blobs, block_tuples), reverse=True) [:3]

        # populate array of digits
        digits = []
        for i in range(2):
            length, idx = top_three[i]
            _, post_idx = top_three[i+1]
            if i == 1:
                digits.append(color[self.VERT_PADDING:num_rows-self.VERT_PADDING, idx+length - self.HZ_PADDING:post_idx-10 + self.HZ_PADDING])
            else: 
                digits.append(color[self.VERT_PADDING:num_rows-self.VERT_PADDING, idx+length - self.HZ_PADDING:post_idx + self.HZ_PADDING])

        idx_3, len_3 = top_three[2]
        digits.append(color[ self.VERT_PADDING:num_rows-self.VERT_PADDING, idx_3+len_3 - self.HZ_PADDING:num_cols - self.HZ_PADDING])
        return digits

    def predict(self, digit):
        img = transforms.Compose([transforms.ToTensor(),
                                    transforms.Resize((28, 28)),
                                    #transforms.Grayscale(1),
                                    transforms.Normalize((0.5,), (0.5,))])(digit)

        input_img = img.reshape(-1, 784) # grayscale color channel transforms.Grayscale

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
            output = model.predict(input_img) # needs type torch.Tensor

        return torch.argmax(output).item()
    
    